#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTranslationScaleM = 0.003;
constexpr double kRotationScaleRad = 0.5 * kPi / 180.0;
constexpr double kHuberDelta = 1.5;

using Matrix4d = Eigen::Matrix4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Vector12d = Eigen::Matrix<double, 12, 1>;

struct Sample {
  std::string name;
  Matrix4d base_to_gripper{Matrix4d::Identity()};
  Matrix4d camera_to_board{Matrix4d::Identity()};
  double reprojection_px{0.0};
  int corners{0};
};

struct Stats {
  double mean{0.0};
  double median{0.0};
  double maximum{0.0};
};

struct Metrics {
  Stats translation_mm;
  Stats rotation_deg;
};

struct Solution {
  Matrix4d gripper_to_camera{Matrix4d::Identity()};
  Matrix4d base_to_board{Matrix4d::Identity()};
  std::vector<double> weights;
  int iterations{0};
  double objective{std::numeric_limits<double>::infinity()};
};

struct Scenario {
  std::string name;
  Solution solution;
  std::vector<int> active;
  std::vector<int> excluded;
  Metrics training;
  Metrics kfold;
  double score{std::numeric_limits<double>::infinity()};
};

struct Options {
  fs::path dataset;
  fs::path output;
  fs::path current_yaml;
  int max_exclusions{5};
  int folds{5};
};

std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (ch == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else if (ch != '\r') {
      field.push_back(ch);
    }
  }
  fields.push_back(field);
  return fields;
}

double degrees(double radians) { return radians * 180.0 / kPi; }

Eigen::Matrix3d projectRotation(const Eigen::Matrix3d &matrix) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d u = svd.matrixU();
    u.col(2) *= -1.0;
    rotation = u * svd.matrixV().transpose();
  }
  return rotation;
}

Eigen::Matrix3d expRotation(const Eigen::Vector3d &omega) {
  const double angle = omega.norm();
  if (angle < 1e-14) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
}

Eigen::Vector3d logRotation(const Eigen::Matrix3d &input) {
  const Eigen::AngleAxisd angle_axis(projectRotation(input));
  if (!std::isfinite(angle_axis.angle()) || std::abs(angle_axis.angle()) < 1e-14) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

Matrix4d perturb(const Matrix4d &transform, const Eigen::Matrix<double, 6, 1> &delta) {
  Matrix4d increment = Matrix4d::Identity();
  increment.block<3, 3>(0, 0) = expRotation(delta.tail<3>() * kPi / 180.0);
  increment.block<3, 1>(0, 3) = delta.head<3>() * 0.001;
  return increment * transform;
}

Vector6d residual(const Sample &sample, const Matrix4d &gripper_to_camera,
                  const Matrix4d &base_to_board) {
  const Matrix4d error = base_to_board.inverse() * sample.base_to_gripper *
                         gripper_to_camera * sample.camera_to_board;
  Vector6d value;
  value.head<3>() = error.block<3, 1>(0, 3) / kTranslationScaleM;
  value.tail<3>() = logRotation(error.block<3, 3>(0, 0)) / kRotationScaleRad;
  return value;
}

double robustCost(double sample_norm) {
  if (sample_norm <= kHuberDelta) {
    return 0.5 * sample_norm * sample_norm;
  }
  return kHuberDelta * (sample_norm - 0.5 * kHuberDelta);
}

double huberWeight(double sample_norm) {
  return sample_norm <= kHuberDelta ? 1.0 : kHuberDelta / sample_norm;
}

double objective(const std::vector<Sample> &samples, const std::vector<int> &indices,
                 const Matrix4d &gripper_to_camera, const Matrix4d &base_to_board) {
  double cost = 0.0;
  for (const int index : indices) {
    cost += robustCost(residual(samples[index], gripper_to_camera, base_to_board).norm() /
                       std::sqrt(6.0));
  }
  return cost;
}

Matrix4d averageTransforms(const std::vector<Matrix4d> &transforms) {
  if (transforms.empty()) {
    throw std::runtime_error("cannot average an empty transform set");
  }
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix4d quaternion_accumulator = Eigen::Matrix4d::Zero();
  for (const auto &transform : transforms) {
    translation += transform.block<3, 1>(0, 3);
    Eigen::Quaterniond q(projectRotation(transform.block<3, 3>(0, 0)));
    if (q.w() < 0.0) {
      q.coeffs() *= -1.0;
    }
    const Eigen::Vector4d coeff(q.w(), q.x(), q.y(), q.z());
    quaternion_accumulator += coeff * coeff.transpose();
  }
  translation /= static_cast<double>(transforms.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(quaternion_accumulator);
  const Eigen::Vector4d qv = eig.eigenvectors().col(3);
  Eigen::Quaterniond q(qv[0], qv[1], qv[2], qv[3]);
  q.normalize();
  Matrix4d result = Matrix4d::Identity();
  result.block<3, 3>(0, 0) = q.toRotationMatrix();
  result.block<3, 1>(0, 3) = translation;
  return result;
}

Matrix4d estimateBoard(const std::vector<Sample> &samples, const std::vector<int> &indices,
                       const Matrix4d &gripper_to_camera) {
  std::vector<Matrix4d> board_poses;
  board_poses.reserve(indices.size());
  for (const int index : indices) {
    board_poses.push_back(samples[index].base_to_gripper * gripper_to_camera *
                          samples[index].camera_to_board);
  }
  return averageTransforms(board_poses);
}

Solution robustSolve(const std::vector<Sample> &samples, const std::vector<int> &indices,
                     const Matrix4d &initial_gripper_to_camera) {
  if (indices.size() < 8) {
    throw std::runtime_error("robust hand-eye solve requires at least 8 samples");
  }
  Solution solution;
  solution.gripper_to_camera = initial_gripper_to_camera;
  solution.base_to_board = estimateBoard(samples, indices, solution.gripper_to_camera);
  double lambda = 1e-3;
  double current_cost = objective(samples, indices, solution.gripper_to_camera,
                                  solution.base_to_board);
  constexpr double epsilon = 1e-4;

  for (int iteration = 0; iteration < 40; ++iteration) {
    Matrix12d normal = Matrix12d::Zero();
    Vector12d gradient = Vector12d::Zero();
    for (const int index : indices) {
      const Vector6d r = residual(samples[index], solution.gripper_to_camera,
                                  solution.base_to_board);
      const double weight = huberWeight(r.norm() / std::sqrt(6.0));
      Eigen::Matrix<double, 6, 12> jacobian;
      for (int column = 0; column < 12; ++column) {
        Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
        delta[column % 6] = epsilon;
        Matrix4d x_plus = solution.gripper_to_camera;
        Matrix4d b_plus = solution.base_to_board;
        Matrix4d x_minus = solution.gripper_to_camera;
        Matrix4d b_minus = solution.base_to_board;
        if (column < 6) {
          x_plus = perturb(x_plus, delta);
          x_minus = perturb(x_minus, -delta);
        } else {
          b_plus = perturb(b_plus, delta);
          b_minus = perturb(b_minus, -delta);
        }
        jacobian.col(column) =
            (residual(samples[index], x_plus, b_plus) -
             residual(samples[index], x_minus, b_minus)) /
            (2.0 * epsilon);
      }
      normal.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * r;
    }

    bool accepted = false;
    Vector12d accepted_delta = Vector12d::Zero();
    for (int attempt = 0; attempt < 8; ++attempt) {
      Matrix12d damped = normal;
      damped.diagonal().array() += lambda * normal.diagonal().array().max(1e-9);
      const Vector12d delta = -damped.ldlt().solve(gradient);
      if (!delta.allFinite()) {
        lambda *= 10.0;
        continue;
      }
      const Matrix4d candidate_x = perturb(solution.gripper_to_camera, delta.head<6>());
      const Matrix4d candidate_b = perturb(solution.base_to_board, delta.tail<6>());
      const double candidate_cost = objective(samples, indices, candidate_x, candidate_b);
      if (candidate_cost < current_cost) {
        solution.gripper_to_camera = candidate_x;
        solution.base_to_board = candidate_b;
        current_cost = candidate_cost;
        accepted_delta = delta;
        lambda = std::max(1e-9, lambda * 0.3);
        accepted = true;
        break;
      }
      lambda *= 10.0;
    }
    solution.iterations = iteration + 1;
    if (!accepted || accepted_delta.norm() < 1e-7) {
      break;
    }
  }

  solution.objective = current_cost;
  solution.weights.assign(samples.size(), 0.0);
  for (const int index : indices) {
    const Vector6d r = residual(samples[index], solution.gripper_to_camera,
                                solution.base_to_board);
    solution.weights[index] = huberWeight(r.norm() / std::sqrt(6.0));
  }
  return solution;
}

Stats calculateStats(std::vector<double> values) {
  Stats stats;
  if (values.empty()) {
    return stats;
  }
  stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  stats.maximum = *std::max_element(values.begin(), values.end());
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  stats.median = values.size() % 2 == 0 ? (values[middle - 1] + values[middle]) * 0.5
                                         : values[middle];
  return stats;
}

Metrics evaluate(const std::vector<Sample> &samples, const std::vector<int> &indices,
                 const Matrix4d &gripper_to_camera, const Matrix4d &base_to_board) {
  std::vector<double> translations;
  std::vector<double> rotations;
  for (const int index : indices) {
    const Matrix4d error = base_to_board.inverse() * samples[index].base_to_gripper *
                           gripper_to_camera * samples[index].camera_to_board;
    translations.push_back(error.block<3, 1>(0, 3).norm() * 1000.0);
    rotations.push_back(degrees(logRotation(error.block<3, 3>(0, 0)).norm()));
  }
  return {calculateStats(translations), calculateStats(rotations)};
}

Metrics kfoldEvaluate(const std::vector<Sample> &samples, const std::vector<int> &active,
                      int folds, const Matrix4d &initial_gripper_to_camera) {
  std::vector<double> translations;
  std::vector<double> rotations;
  folds = std::min(folds, static_cast<int>(active.size() / 6));
  for (int fold = 0; fold < folds; ++fold) {
    std::vector<int> training;
    std::vector<int> validation;
    for (std::size_t position = 0; position < active.size(); ++position) {
      ((static_cast<int>(position) % folds == fold) ? validation : training)
          .push_back(active[position]);
    }
    const Solution solution = robustSolve(samples, training, initial_gripper_to_camera);
    for (const int index : validation) {
      const Matrix4d error = solution.base_to_board.inverse() * samples[index].base_to_gripper *
                             solution.gripper_to_camera * samples[index].camera_to_board;
      translations.push_back(error.block<3, 1>(0, 3).norm() * 1000.0);
      rotations.push_back(degrees(logRotation(error.block<3, 3>(0, 0)).norm()));
    }
  }
  return {calculateStats(translations), calculateStats(rotations)};
}

double qualityScore(const Metrics &training, const Metrics &kfold) {
  return training.translation_mm.median / 3.0 + training.translation_mm.maximum / 8.0 +
         training.rotation_deg.median / 0.5 + training.rotation_deg.maximum / 1.5 +
         kfold.translation_mm.median / 3.0 + kfold.translation_mm.maximum / 8.0 +
         kfold.rotation_deg.median / 0.5 + kfold.rotation_deg.maximum / 1.5;
}

std::vector<int> allIndices(std::size_t count) {
  std::vector<int> indices(count);
  std::iota(indices.begin(), indices.end(), 0);
  return indices;
}

int worstIndex(const std::vector<Sample> &samples, const std::vector<int> &active,
               const Solution &solution) {
  return *std::max_element(active.begin(), active.end(), [&](int left, int right) {
    return residual(samples[left], solution.gripper_to_camera, solution.base_to_board).norm() <
           residual(samples[right], solution.gripper_to_camera, solution.base_to_board).norm();
  });
}

Scenario evaluateScenario(const std::string &name, const std::vector<Sample> &samples,
                          const std::vector<int> &active, const std::vector<int> &excluded,
                          int folds, const Matrix4d &initial_gripper_to_camera) {
  Scenario scenario;
  scenario.name = name;
  scenario.active = active;
  scenario.excluded = excluded;
  scenario.solution = robustSolve(samples, active, initial_gripper_to_camera);
  scenario.training = evaluate(samples, active, scenario.solution.gripper_to_camera,
                               scenario.solution.base_to_board);
  scenario.kfold = kfoldEvaluate(samples, active, folds, initial_gripper_to_camera);
  scenario.score = qualityScore(scenario.training, scenario.kfold);
  return scenario;
}

std::pair<std::vector<int>, std::vector<int>> partitionByName(
    const std::vector<Sample> &samples, const std::set<std::string> &excluded_names) {
  std::vector<int> active;
  std::vector<int> excluded;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    (excluded_names.count(samples[index].name) ? excluded : active)
        .push_back(static_cast<int>(index));
  }
  if (excluded.size() != excluded_names.size()) {
    throw std::runtime_error("a predefined high-influence sample is missing");
  }
  return {active, excluded};
}

std::vector<Scenario> buildScenarios(const std::vector<Sample> &samples, int max_exclusions,
                                     int folds, const Matrix4d &initial_gripper_to_camera) {
  std::vector<Scenario> scenarios;
  std::vector<int> active = allIndices(samples.size());
  std::vector<int> excluded;
  for (int count = 0; count <= max_exclusions; ++count) {
    const Scenario scenario = evaluateScenario(
        "robust_trim_" + std::to_string(count), samples, active, excluded, folds,
        initial_gripper_to_camera);
    scenarios.push_back(scenario);
    if (count < max_exclusions) {
      const int worst = worstIndex(samples, active, scenario.solution);
      excluded.push_back(worst);
      active.erase(std::remove(active.begin(), active.end(), worst), active.end());
    }
  }
  for (const auto &[name, excluded_names] : std::vector<std::pair<std::string, std::set<std::string>>>{
           {"known_influence_3", {"img_008.png", "img_009.png", "img_016.png"}},
           {"known_influence_5", {"img_002.png", "img_008.png", "img_009.png",
                                   "img_013.png", "img_016.png"}}}) {
    const auto [known_active, known_excluded] = partitionByName(samples, excluded_names);
    scenarios.push_back(evaluateScenario(name, samples, known_active, known_excluded, folds,
                                         initial_gripper_to_camera));
  }
  return scenarios;
}

std::map<std::string, std::string> readPoseHeader(const fs::path &csv_path,
                                                  std::vector<std::vector<std::string>> &rows) {
  std::ifstream input(csv_path);
  if (!input) {
    throw std::runtime_error("cannot open " + csv_path.string());
  }
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("empty CSV: " + csv_path.string());
  }
  const auto header = splitCsv(line);
  std::map<std::string, std::string> columns;
  for (std::size_t i = 0; i < header.size(); ++i) {
    columns[header[i]] = std::to_string(i);
  }
  while (std::getline(input, line)) {
    if (!line.empty()) {
      rows.push_back(splitCsv(line));
    }
  }
  return columns;
}

int columnIndex(const std::map<std::string, std::string> &columns, const std::string &name) {
  const auto found = columns.find(name);
  if (found == columns.end()) {
    throw std::runtime_error("robot_poses.csv missing column: " + name);
  }
  return std::stoi(found->second);
}

Matrix4d parseMatrix(const YAML::Node &node, const std::string &field) {
  if (!node || node.size() != 4) {
    throw std::runtime_error("invalid matrix field: " + field);
  }
  Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = node[row][col].as<double>();
    }
  }
  return matrix;
}

std::vector<Sample> loadSamples(const fs::path &cache_path) {
  const YAML::Node root = YAML::LoadFile(cache_path.string());
  const YAML::Node rows = root["samples"];
  if (!rows || !rows.IsSequence()) {
    throw std::runtime_error("sample cache has no samples sequence: " + cache_path.string());
  }
  std::vector<Sample> samples;
  for (const auto &row : rows) {
    Sample sample;
    sample.name = row["name"].as<std::string>();
    sample.base_to_gripper = parseMatrix(row["T_base_to_gripper"], "T_base_to_gripper");
    sample.camera_to_board = parseMatrix(row["T_camera_to_board"], "T_camera_to_board");
    sample.reprojection_px = row["reprojection_px"].as<double>();
    sample.corners = row["corners"].as<int>();
    samples.push_back(sample);
  }
  return samples;
}

Matrix4d loadTransform(const fs::path &path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node matrix = root["T_camera_to_gripper"];
  if (!matrix || matrix.size() != 4) {
    throw std::runtime_error("missing T_camera_to_gripper in " + path.string());
  }
  Matrix4d transform;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      transform(row, col) = matrix[row][col].as<double>();
    }
  }
  return transform;
}

YAML::Node matrixNode(const Matrix4d &matrix) {
  YAML::Node outer(YAML::NodeType::Sequence);
  for (int row = 0; row < 4; ++row) {
    YAML::Node values(YAML::NodeType::Sequence);
    for (int col = 0; col < 4; ++col) {
      values.push_back(matrix(row, col));
    }
    outer.push_back(values);
  }
  return outer;
}

YAML::Node statsNode(const Stats &stats) {
  YAML::Node node;
  node["mean"] = stats.mean;
  node["median"] = stats.median;
  node["max"] = stats.maximum;
  return node;
}

YAML::Node metricsNode(const Metrics &metrics) {
  YAML::Node node;
  node["translation_mm"] = statsNode(metrics.translation_mm);
  node["rotation_deg"] = statsNode(metrics.rotation_deg);
  return node;
}

void saveReport(const Options &options, const std::vector<Sample> &samples,
                const Matrix4d &current, const Metrics &current_metrics,
                const std::vector<Scenario> &scenarios, const Scenario &selected) {
  YAML::Node root;
  root["schema_version"] = 1;
  root["status"] = "offline_candidate_only";
  root["validated"] = false;
  root["execution_allowed"] = false;
  root["motion_commands_published"] = false;
  const YAML::Node cache = YAML::LoadFile(options.dataset.string());
  root["source_dataset"] = cache["source_dataset"].as<std::string>();
  root["source_pnp_cache"] = options.dataset.string();
  root["current_calibration"] = options.current_yaml.string();
  root["algorithm"] = "joint_SE3_Huber_IRLS_LM";
  root["residual_scaling"]["translation_mm"] = kTranslationScaleM * 1000.0;
  root["residual_scaling"]["rotation_deg"] = degrees(kRotationScaleRad);
  root["huber_delta"] = kHuberDelta;
  root["sample_count"] = samples.size();
  root["minimum_retained_samples"] = 36;
  root["current_matrix_metrics_all_samples"] = metricsNode(current_metrics);

  const Matrix4d delta = current.inverse() * selected.solution.gripper_to_camera;
  root["selected_scenario"] = selected.name;
  root["excluded_samples"] = YAML::Node(YAML::NodeType::Sequence);
  for (const int index : selected.excluded) {
    root["excluded_samples"].push_back(samples[index].name);
  }
  root["T_camera_to_gripper"] = matrixNode(selected.solution.gripper_to_camera);
  root["T_gripper_to_camera"] = matrixNode(selected.solution.gripper_to_camera.inverse());
  root["delta_from_current"]["translation_mm"] = delta.block<3, 1>(0, 3).norm() * 1000.0;
  root["delta_from_current"]["rotation_deg"] = degrees(logRotation(delta.block<3, 3>(0, 0)).norm());
  root["quality"]["training_retained"] = metricsNode(selected.training);
  root["quality"]["kfold_retained"] = metricsNode(selected.kfold);
  root["quality"]["score"] = selected.score;
  if (!selected.excluded.empty()) {
    root["quality"]["excluded_against_candidate"] = metricsNode(
        evaluate(samples, selected.excluded, selected.solution.gripper_to_camera,
                 selected.solution.base_to_board));
  }

  YAML::Node scenario_nodes(YAML::NodeType::Sequence);
  for (const auto &scenario : scenarios) {
    YAML::Node node;
    node["name"] = scenario.name;
    node["retained_samples"] = scenario.active.size();
    node["excluded_samples"] = YAML::Node(YAML::NodeType::Sequence);
    for (const int index : scenario.excluded) {
      node["excluded_samples"].push_back(samples[index].name);
    }
    node["training"] = metricsNode(scenario.training);
    node["kfold"] = metricsNode(scenario.kfold);
    node["score"] = scenario.score;
    node["iterations"] = scenario.solution.iterations;
    scenario_nodes.push_back(node);
  }
  root["scenarios"] = scenario_nodes;
  root["acceptance"]["accepted"] = false;
  const bool training_translation = selected.training.translation_mm.median <= 3.0 &&
                                    selected.training.translation_mm.maximum <= 8.0;
  const bool training_rotation = selected.training.rotation_deg.median <= 0.5 &&
                                 selected.training.rotation_deg.maximum <= 1.5;
  const bool kfold_translation = selected.kfold.translation_mm.median <= 3.0 &&
                                 selected.kfold.translation_mm.maximum <= 8.0;
  const bool kfold_rotation = selected.kfold.rotation_deg.median <= 0.5 &&
                              selected.kfold.rotation_deg.maximum <= 1.5;
  root["acceptance"]["checks"]["minimum_retained_samples"] = selected.active.size() >= 36;
  root["acceptance"]["checks"]["fixed_board_translation"] = training_translation;
  root["acceptance"]["checks"]["fixed_board_rotation"] = training_rotation;
  root["acceptance"]["checks"]["kfold_translation"] = kfold_translation;
  root["acceptance"]["checks"]["kfold_rotation"] = kfold_rotation;
  root["acceptance"]["reason"] =
      "candidate remains disabled until every offline check and an independent blind entry validation pass";
  root["known_systematic"]["depth_vs_pnp_signed_offset_mm"] = -9.502643;
  root["known_systematic"]["note"] =
      "this hand-eye candidate cannot by itself remove the measured L515 depth/PnP offset";

  fs::create_directories(options.output.parent_path());
  std::ofstream output(options.output);
  if (!output) {
    throw std::runtime_error("cannot write " + options.output.string());
  }
  output << std::setprecision(15) << root;
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value after " + arg);
      }
      return argv[++i];
    };
    if (arg == "--dataset") {
      options.dataset = value();
    } else if (arg == "--output") {
      options.output = value();
    } else if (arg == "--current-yaml") {
      options.current_yaml = value();
    } else if (arg == "--max-exclusions") {
      options.max_exclusions = std::stoi(value());
    } else if (arg == "--folds") {
      options.folds = std::stoi(value());
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: robust_handeye_analyzer --dataset DIR --current-yaml FILE "
                   "--output FILE [--max-exclusions 5] [--folds 5]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (options.dataset.empty() || options.output.empty() || options.current_yaml.empty()) {
    throw std::runtime_error("--dataset, --output and --current-yaml are required");
  }
  if (options.max_exclusions < 0 || options.max_exclusions > 8 || options.folds < 2) {
    throw std::runtime_error("invalid exclusion or fold count");
  }
  return options;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const std::vector<Sample> samples = loadSamples(options.dataset);
    std::cout << "Loaded " << samples.size() << " synchronized ChArUco samples\n";

    const Matrix4d current = loadTransform(options.current_yaml);
    const auto indices = allIndices(samples.size());
    const Matrix4d current_board = estimateBoard(samples, indices, current);
    const Metrics current_metrics = evaluate(samples, indices, current, current_board);
    std::cout << std::fixed << std::setprecision(3)
              << "Current matrix: board median/max "
              << current_metrics.translation_mm.median << "/"
              << current_metrics.translation_mm.maximum << " mm, "
              << current_metrics.rotation_deg.median << "/"
              << current_metrics.rotation_deg.maximum << " deg\n";

    const std::vector<Scenario> scenarios =
        buildScenarios(samples, options.max_exclusions, options.folds, current);
    for (const auto &scenario : scenarios) {
      std::cout << scenario.name << ": retained=" << scenario.active.size()
                << " train=" << scenario.training.translation_mm.median << "/"
                << scenario.training.translation_mm.maximum << " mm, "
                << scenario.training.rotation_deg.median << "/"
                << scenario.training.rotation_deg.maximum << " deg; cv="
                << scenario.kfold.translation_mm.median << "/"
                << scenario.kfold.translation_mm.maximum << " mm, "
                << scenario.kfold.rotation_deg.median << "/"
                << scenario.kfold.rotation_deg.maximum << " deg; score="
                << scenario.score << "\n";
    }

    const auto selected_it = std::min_element(
        scenarios.begin(), scenarios.end(), [](const Scenario &left, const Scenario &right) {
          const bool left_eligible = left.active.size() >= 36;
          const bool right_eligible = right.active.size() >= 36;
          if (left_eligible != right_eligible) {
            return left_eligible;
          }
          return left.score < right.score;
        });
    if (selected_it == scenarios.end() || selected_it->active.size() < 36) {
      throw std::runtime_error("no robust scenario retains the required 36 samples");
    }
    saveReport(options, samples, current, current_metrics, scenarios, *selected_it);
    std::cout << "Offline-only candidate: " << options.output << "\n"
              << "Selected " << selected_it->name
              << "; validated=false, execution_allowed=false\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "robust_handeye_analyzer: " << error.what() << "\n";
    return 1;
  }
}
