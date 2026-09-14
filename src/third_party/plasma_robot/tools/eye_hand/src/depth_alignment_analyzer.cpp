#include "plasma_eye_hand/depth_alignment.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Options
{
  fs::path dataset;
  fs::path output;
};

struct SampleMetadata
{
  int corners = 0;
  double reprojection_px = 0.0;
  double board_distance_m = 0.0;
  double stamp_delta_ms = 0.0;
  int median_depth_frames = 0;
  std::string target;
};

struct Dataset
{
  double raw_depth_unit_m = 0.001;
  Eigen::Isometry3d factory_color_from_depth = Eigen::Isometry3d::Identity();
  std::vector<plasma_eye_hand::DepthPlaneSample> samples;
  std::vector<SampleMetadata> metadata;
};

struct ScalarStats
{
  double mean = 0.0;
  double median = 0.0;
  double maximum = 0.0;
};

struct FoldResult
{
  std::string held_out;
  plasma_eye_hand::DepthAlignmentFitResult fit;
  plasma_eye_hand::DepthResidualStats held_out_residuals;
  double scale_difference = 0.0;
  double bias_difference_m = 0.0;
  double extrinsic_translation_difference_m = 0.0;
  double extrinsic_rotation_difference_rad = 0.0;
};

struct PlaneDiagnostic
{
  Eigen::Vector3d depth_normal_color = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d pnp_normal_color = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d depth_centroid_color = Eigen::Vector3d::Zero();
  double normal_error_rad = 0.0;
  plasma_eye_hand::DepthResidualStats internal_residuals;
};

double degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

Eigen::Isometry3d parseTransform(const YAML::Node & node, const std::string & field)
{
  if (!node || !node.IsSequence() || node.size() != 4) {
    throw std::runtime_error("invalid transform: " + field);
  }
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    if (!node[row].IsSequence() || node[row].size() != 4) {
      throw std::runtime_error("invalid transform row: " + field);
    }
    for (int column = 0; column < 4; ++column) {
      matrix(row, column) = node[row][column].as<double>();
    }
  }
  return Eigen::Isometry3d(matrix);
}

Dataset loadDataset(const fs::path & path)
{
  const YAML::Node root = YAML::LoadFile(path.string());
  if (!root["samples"] || !root["samples"].IsSequence()) {
    throw std::runtime_error("dataset has no samples sequence");
  }
  if (root["camera_parameters_modified"].as<bool>()) {
    throw std::runtime_error("dataset reports modified camera parameters");
  }
  Dataset dataset;
  dataset.raw_depth_unit_m = root["raw_depth_unit_hint_m"].as<double>();
  dataset.factory_color_from_depth = parseTransform(
    root["T_color_from_depth_factory"], "T_color_from_depth_factory");
  for (const auto & node : root["samples"]) {
    plasma_eye_hand::DepthPlaneSample sample;
    sample.name = node["name"].as<std::string>();
    sample.color_from_board = parseTransform(
      node["T_camera_to_board"], sample.name + ".T_camera_to_board");
    const YAML::Node rays = node["depth_ray_raw"];
    if (!rays || !rays.IsSequence()) {
      throw std::runtime_error("sample has no depth rays: " + sample.name);
    }
    sample.rays.reserve(rays.size());
    for (const auto & ray : rays) {
      if (!ray.IsSequence() || ray.size() != 3) {
        throw std::runtime_error("invalid depth ray in " + sample.name);
      }
      sample.rays.push_back({ray[0].as<double>(), ray[1].as<double>(), ray[2].as<double>()});
    }
    SampleMetadata metadata;
    metadata.corners = node["corners"].as<int>();
    metadata.reprojection_px = node["reprojection_px"].as<double>();
    metadata.board_distance_m = node["board_distance_m"].as<double>();
    metadata.stamp_delta_ms = std::abs(
      node["color_stamp"].as<double>() - node["depth_stamp"].as<double>()) * 1000.0;
    metadata.median_depth_frames = node["median_depth_frames"].as<int>();
    metadata.target = node["target"].as<std::string>();
    dataset.samples.push_back(std::move(sample));
    dataset.metadata.push_back(std::move(metadata));
  }
  if (dataset.samples.size() < 9) {
    throw std::runtime_error("offline depth candidate requires at least nine groups");
  }
  return dataset;
}

ScalarStats stats(std::vector<double> values)
{
  ScalarStats result;
  if (values.empty()) {
    return result;
  }
  result.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  result.maximum = *std::max_element(values.begin(), values.end());
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  result.median = values.size() % 2 == 0 ?
    0.5 * (values[middle - 1] + values[middle]) : values[middle];
  return result;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 0 ?
         0.5 * (values[middle - 1] + values[middle]) : values[middle];
}

plasma_eye_hand::DepthResidualStats residualStats(const std::vector<double> & residuals)
{
  plasma_eye_hand::DepthResidualStats result;
  result.count = residuals.size();
  if (residuals.empty()) {
    return result;
  }
  std::vector<double> absolute;
  absolute.reserve(residuals.size());
  double signed_sum = 0.0;
  double absolute_sum = 0.0;
  double squared_sum = 0.0;
  for (const double residual : residuals) {
    const double value = std::abs(residual);
    signed_sum += residual;
    absolute_sum += value;
    squared_sum += residual * residual;
    result.max_abs_m = std::max(result.max_abs_m, value);
    absolute.push_back(value);
  }
  const double count = static_cast<double>(residuals.size());
  result.mean_signed_m = signed_sum / count;
  result.median_signed_m = median(residuals);
  result.mean_abs_m = absolute_sum / count;
  result.median_abs_m = median(absolute);
  result.rms_m = std::sqrt(squared_sum / count);
  return result;
}

PlaneDiagnostic diagnosePlane(
  const plasma_eye_hand::DepthPlaneSample & sample,
  const Eigen::Isometry3d & factory_color_from_depth,
  double raw_depth_unit_m)
{
  const plasma_eye_hand::DepthAlignmentParameters baseline;
  std::vector<Eigen::Vector3d> points;
  points.reserve(sample.rays.size());
  PlaneDiagnostic result;
  for (const auto & ray : sample.rays) {
    const Eigen::Vector3d point = plasma_eye_hand::correctedDepthPoint(
      ray, factory_color_from_depth, raw_depth_unit_m, baseline);
    points.push_back(point);
    result.depth_centroid_color += point;
  }
  result.depth_centroid_color /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : points) {
    const Eigen::Vector3d centered = point - result.depth_centroid_color;
    covariance.noalias() += centered * centered.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("depth plane eigensolver failed for " + sample.name);
  }
  result.depth_normal_color = solver.eigenvectors().col(0).normalized();
  result.pnp_normal_color = sample.color_from_board.linear().col(2).normalized();
  if (result.depth_normal_color.dot(result.pnp_normal_color) < 0.0) {
    result.depth_normal_color *= -1.0;
  }
  const double cosine = std::clamp(
    result.depth_normal_color.dot(result.pnp_normal_color), -1.0, 1.0);
  result.normal_error_rad = std::acos(cosine);
  std::vector<double> internal_residuals;
  internal_residuals.reserve(points.size());
  for (const auto & point : points) {
    internal_residuals.push_back(
      result.depth_normal_color.dot(point - result.depth_centroid_color));
  }
  result.internal_residuals = residualStats(internal_residuals);
  return result;
}

YAML::Node transformNode(const Eigen::Isometry3d & transform)
{
  YAML::Node outer(YAML::NodeType::Sequence);
  for (int row = 0; row < 4; ++row) {
    YAML::Node values(YAML::NodeType::Sequence);
    for (int column = 0; column < 4; ++column) {
      values.push_back(transform.matrix()(row, column));
    }
    outer.push_back(values);
  }
  return outer;
}

YAML::Node residualNode(const plasma_eye_hand::DepthResidualStats & stats_value)
{
  YAML::Node node;
  node["point_count"] = stats_value.count;
  node["mean_signed_mm"] = stats_value.mean_signed_m * 1000.0;
  node["median_signed_mm"] = stats_value.median_signed_m * 1000.0;
  node["mean_abs_mm"] = stats_value.mean_abs_m * 1000.0;
  node["median_abs_mm"] = stats_value.median_abs_m * 1000.0;
  node["rms_mm"] = stats_value.rms_m * 1000.0;
  node["max_abs_mm"] = stats_value.max_abs_m * 1000.0;
  return node;
}

YAML::Node scalarStatsNode(const ScalarStats & value, double multiplier)
{
  YAML::Node node;
  node["mean"] = value.mean * multiplier;
  node["median"] = value.median * multiplier;
  node["max"] = value.maximum * multiplier;
  return node;
}

std::vector<FoldResult> leaveOneGroupOut(
  const Dataset & dataset,
  const plasma_eye_hand::DepthAlignmentFitResult & full_fit)
{
  std::vector<FoldResult> folds;
  const Eigen::Isometry3d full_extrinsic =
    full_fit.parameters.correction * dataset.factory_color_from_depth;
  for (std::size_t held_out = 0; held_out < dataset.samples.size(); ++held_out) {
    std::vector<plasma_eye_hand::DepthPlaneSample> training;
    training.reserve(dataset.samples.size() - 1);
    for (std::size_t index = 0; index < dataset.samples.size(); ++index) {
      if (index != held_out) {
        training.push_back(dataset.samples[index]);
      }
    }
    FoldResult fold;
    fold.held_out = dataset.samples[held_out].name;
    fold.fit = plasma_eye_hand::fitDepthAlignment(
      training, dataset.factory_color_from_depth, dataset.raw_depth_unit_m);
    fold.held_out_residuals = plasma_eye_hand::evaluateDepthAlignment(
      {dataset.samples[held_out]}, dataset.factory_color_from_depth,
      dataset.raw_depth_unit_m, fold.fit.parameters);
    fold.scale_difference = std::abs(
      fold.fit.parameters.depth_scale_factor - full_fit.parameters.depth_scale_factor);
    fold.bias_difference_m = std::abs(
      fold.fit.parameters.ray_bias_m - full_fit.parameters.ray_bias_m);
    const Eigen::Isometry3d fold_extrinsic =
      fold.fit.parameters.correction * dataset.factory_color_from_depth;
    const Eigen::Isometry3d difference = full_extrinsic.inverse() * fold_extrinsic;
    fold.extrinsic_translation_difference_m = difference.translation().norm();
    fold.extrinsic_rotation_difference_rad = rotationAngle(difference.linear());
    folds.push_back(std::move(fold));
  }
  return folds;
}

void saveReport(
  const Options & options,
  const Dataset & dataset,
  const plasma_eye_hand::DepthResidualStats & baseline,
  const plasma_eye_hand::DepthAlignmentFitResult & full_fit,
  const std::vector<FoldResult> & folds)
{
  const Eigen::Isometry3d candidate_extrinsic =
    full_fit.parameters.correction * dataset.factory_color_from_depth;
  std::vector<double> loo_medians;
  std::vector<double> loo_rms;
  std::vector<double> scale_spread;
  std::vector<double> bias_spread;
  std::vector<double> translation_spread;
  std::vector<double> rotation_spread;
  for (const auto & fold : folds) {
    loo_medians.push_back(fold.held_out_residuals.median_abs_m);
    loo_rms.push_back(fold.held_out_residuals.rms_m);
    scale_spread.push_back(fold.scale_difference);
    bias_spread.push_back(fold.bias_difference_m);
    translation_spread.push_back(fold.extrinsic_translation_difference_m);
    rotation_spread.push_back(fold.extrinsic_rotation_difference_rad);
  }
  const ScalarStats loo_median_stats = stats(loo_medians);
  const ScalarStats loo_rms_stats = stats(loo_rms);
  const ScalarStats scale_stats = stats(scale_spread);
  const ScalarStats bias_stats = stats(bias_spread);
  const ScalarStats translation_stats = stats(translation_spread);
  const ScalarStats rotation_stats = stats(rotation_spread);

  YAML::Node root;
  root["schema_version"] = 1;
  root["status"] = "offline_candidate_only";
  root["validated"] = false;
  root["execution_allowed"] = false;
  root["motion_commands_published"] = false;
  root["camera_parameters_modified"] = false;
  root["source_dataset"] = fs::absolute(options.dataset).string();
  root["algorithm"] = "joint_depth_scale_ray_bias_SE3_Huber_LM";
  root["model"]["depth_point"] =
    "[x,y,1]*(raw*unit*scale) + normalize([x,y,1])*ray_bias";
  root["model"]["color_point"] =
    "SE3_correction * T_color_from_depth_factory * depth_point";
  root["model"]["residual"] = "signed point-to-ChArUco-PnP-plane distance";
  root["sample_count"] = dataset.samples.size();
  std::size_t ray_count = 0;
  for (const auto & sample : dataset.samples) {
    ray_count += sample.rays.size();
  }
  root["ray_count"] = ray_count;
  root["raw_depth_unit_hint_m"] = dataset.raw_depth_unit_m;
  root["baseline_factory"] = residualNode(baseline);
  root["diagnostics"]["factory_rotation_storage"] =
    "verified column-major from rs2_extrinsics and ROS driver pass-through";
  Eigen::Isometry3d transpose_factory = dataset.factory_color_from_depth;
  transpose_factory.linear() = dataset.factory_color_from_depth.linear().transpose();
  root["diagnostics"]["transposed_factory_rotation_not_candidate"] = residualNode(
    plasma_eye_hand::evaluateDepthAlignment(
      dataset.samples, transpose_factory, dataset.raw_depth_unit_m,
      plasma_eye_hand::DepthAlignmentParameters()));
  root["candidate"]["depth_scale_factor"] = full_fit.parameters.depth_scale_factor;
  root["candidate"]["depth_unit_m_per_raw"] =
    dataset.raw_depth_unit_m * full_fit.parameters.depth_scale_factor;
  root["candidate"]["ray_bias_mm"] = full_fit.parameters.ray_bias_m * 1000.0;
  root["candidate"]["T_color_from_depth_factory"] =
    transformNode(dataset.factory_color_from_depth);
  root["candidate"]["T_factory_correction"] = transformNode(full_fit.parameters.correction);
  root["candidate"]["T_color_from_depth_candidate"] = transformNode(candidate_extrinsic);
  root["candidate"]["correction_translation_mm"] =
    full_fit.parameters.correction.translation().norm() * 1000.0;
  root["candidate"]["correction_rotation_deg"] =
    degrees(rotationAngle(full_fit.parameters.correction.linear()));
  root["candidate"]["fit_iterations"] = full_fit.iterations;
  root["candidate"]["fit_converged"] = full_fit.converged;
  root["candidate"]["normal_condition_number"] = full_fit.normal_condition_number;
  root["candidate"]["all_samples"] = residualNode(full_fit.residuals);

  YAML::Node sample_nodes(YAML::NodeType::Sequence);
  for (std::size_t index = 0; index < dataset.samples.size(); ++index) {
    YAML::Node node;
    node["name"] = dataset.samples[index].name;
    node["target"] = dataset.metadata[index].target;
    node["corners"] = dataset.metadata[index].corners;
    node["reprojection_px"] = dataset.metadata[index].reprojection_px;
    node["board_distance_m"] = dataset.metadata[index].board_distance_m;
    node["color_depth_stamp_delta_ms"] = dataset.metadata[index].stamp_delta_ms;
    node["median_depth_frames"] = dataset.metadata[index].median_depth_frames;
    node["ray_count"] = dataset.samples[index].rays.size();
    node["baseline"] = residualNode(plasma_eye_hand::evaluateDepthAlignment(
        {dataset.samples[index]}, dataset.factory_color_from_depth,
        dataset.raw_depth_unit_m, plasma_eye_hand::DepthAlignmentParameters()));
    node["candidate_all_fit"] = residualNode(plasma_eye_hand::evaluateDepthAlignment(
        {dataset.samples[index]}, dataset.factory_color_from_depth,
        dataset.raw_depth_unit_m, full_fit.parameters));
    node["held_out"] = residualNode(folds[index].held_out_residuals);
    const PlaneDiagnostic plane = diagnosePlane(
      dataset.samples[index], dataset.factory_color_from_depth, dataset.raw_depth_unit_m);
    node["plane_diagnostic"]["depth_self_fit"] = residualNode(plane.internal_residuals);
    node["plane_diagnostic"]["normal_difference_deg"] = degrees(plane.normal_error_rad);
    node["plane_diagnostic"]["depth_normal_color"] = YAML::Load("[]");
    node["plane_diagnostic"]["pnp_normal_color"] = YAML::Load("[]");
    node["plane_diagnostic"]["depth_centroid_color_m"] = YAML::Load("[]");
    for (int axis = 0; axis < 3; ++axis) {
      node["plane_diagnostic"]["depth_normal_color"].push_back(
        plane.depth_normal_color[axis]);
      node["plane_diagnostic"]["pnp_normal_color"].push_back(
        plane.pnp_normal_color[axis]);
      node["plane_diagnostic"]["depth_centroid_color_m"].push_back(
        plane.depth_centroid_color[axis]);
    }
    sample_nodes.push_back(node);
  }
  root["per_sample"] = sample_nodes;

  root["leave_one_group_out"]["group_median_abs_mm"] =
    scalarStatsNode(loo_median_stats, 1000.0);
  root["leave_one_group_out"]["group_rms_mm"] =
    scalarStatsNode(loo_rms_stats, 1000.0);
  YAML::Node fold_nodes(YAML::NodeType::Sequence);
  for (const auto & fold : folds) {
    YAML::Node node;
    node["held_out"] = fold.held_out;
    node["residuals"] = residualNode(fold.held_out_residuals);
    node["fit_iterations"] = fold.fit.iterations;
    node["fit_converged"] = fold.fit.converged;
    node["normal_condition_number"] = fold.fit.normal_condition_number;
    fold_nodes.push_back(node);
  }
  root["leave_one_group_out"]["folds"] = fold_nodes;

  root["leave_one_group_out"]["parameter_stability"]["scale_factor_abs_difference"] =
    scalarStatsNode(scale_stats, 1.0);
  root["leave_one_group_out"]["parameter_stability"]["ray_bias_abs_difference_mm"] =
    scalarStatsNode(bias_stats, 1000.0);
  root["leave_one_group_out"]["parameter_stability"]["extrinsic_translation_difference_mm"] =
    scalarStatsNode(translation_stats, 1000.0);
  root["leave_one_group_out"]["parameter_stability"]["extrinsic_rotation_difference_deg"] =
    scalarStatsNode(rotation_stats, 180.0 / kPi);

  const bool enough_samples = dataset.samples.size() >= 9;
  const bool all_fit_quality = full_fit.residuals.median_abs_m <= 0.001;
  const bool held_out_quality =
    loo_median_stats.median <= 0.002 && loo_median_stats.maximum <= 0.003;
  const bool meaningful_improvement =
    full_fit.residuals.median_abs_m <= 0.5 * baseline.median_abs_m;
  const bool conditioning = std::isfinite(full_fit.normal_condition_number) &&
    full_fit.normal_condition_number <= 1.0e10;
  const bool parameter_stability =
    scale_stats.maximum <= 0.005 && bias_stats.maximum <= 0.005 &&
    translation_stats.maximum <= 0.005 && degrees(rotation_stats.maximum) <= 0.5;
  root["acceptance"]["accepted"] = false;
  root["acceptance"]["offline_fit_quality_passed"] =
    enough_samples && all_fit_quality && held_out_quality && meaningful_improvement &&
    conditioning && parameter_stability;
  root["acceptance"]["checks"]["minimum_nine_groups"] = enough_samples;
  root["acceptance"]["checks"]["all_fit_median_abs_le_1mm"] = all_fit_quality;
  root["acceptance"]["checks"]["loo_group_median_le_2mm_and_max_le_3mm"] = held_out_quality;
  root["acceptance"]["checks"]["median_error_reduced_by_half"] = meaningful_improvement;
  root["acceptance"]["checks"]["normal_condition_le_1e10"] = conditioning;
  root["acceptance"]["checks"]["leave_one_out_parameter_stability"] = parameter_stability;
  root["acceptance"]["reason"] =
    "offline candidate remains disabled until review and an independent blind entry validation pass";

  fs::create_directories(options.output.parent_path());
  std::ofstream stream(options.output);
  if (!stream) {
    throw std::runtime_error("cannot write output: " + options.output.string());
  }
  stream << std::setprecision(15) << root;
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&]() -> std::string {
        if (index + 1 >= argc) {
          throw std::runtime_error("missing value after " + argument);
        }
        return argv[++index];
      };
    if (argument == "--dataset") {
      options.dataset = value();
    } else if (argument == "--output") {
      options.output = value();
    } else if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: depth_alignment_analyzer --dataset dataset.yaml --output candidate.yaml\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.dataset.empty() || options.output.empty()) {
    throw std::runtime_error("--dataset and --output are required");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    const Dataset dataset = loadDataset(options.dataset);
    const plasma_eye_hand::DepthAlignmentParameters baseline_parameters;
    const auto baseline = plasma_eye_hand::evaluateDepthAlignment(
      dataset.samples, dataset.factory_color_from_depth,
      dataset.raw_depth_unit_m, baseline_parameters);
    std::cout << std::fixed << std::setprecision(3)
              << "Loaded " << dataset.samples.size() << " groups; factory plane median/RMS="
              << baseline.median_abs_m * 1000.0 << "/" << baseline.rms_m * 1000.0
              << " mm\n";

    const auto full_fit = plasma_eye_hand::fitDepthAlignment(
      dataset.samples, dataset.factory_color_from_depth, dataset.raw_depth_unit_m);
    std::cout << "Candidate plane median/RMS=" << full_fit.residuals.median_abs_m * 1000.0
              << "/" << full_fit.residuals.rms_m * 1000.0
              << " mm; scale=" << std::setprecision(8)
              << full_fit.parameters.depth_scale_factor
              << ", bias=" << std::setprecision(3)
              << full_fit.parameters.ray_bias_m * 1000.0 << " mm\n";

    const auto folds = leaveOneGroupOut(dataset, full_fit);
    for (const auto & fold : folds) {
      std::cout << fold.held_out << " held-out median/RMS="
                << fold.held_out_residuals.median_abs_m * 1000.0 << "/"
                << fold.held_out_residuals.rms_m * 1000.0 << " mm\n";
    }
    saveReport(options, dataset, baseline, full_fit, folds);
    std::cout << "Offline-only candidate: " << options.output << "\n"
              << "validated=false, execution_allowed=false\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "depth_alignment_analyzer: " << error.what() << "\n";
    return 1;
  }
}
