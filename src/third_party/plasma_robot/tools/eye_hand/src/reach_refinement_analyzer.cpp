#include "plasma_eye_hand/reach_refinement.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

Eigen::Isometry3d matrixFromYaml(const YAML::Node & node, const std::string & key)
{
  if (!node || !node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(key + " must be a 4x4 matrix");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (std::size_t row = 0; row < 4; ++row) {
    if (!node[row].IsSequence() || node[row].size() != 4) {
      throw std::runtime_error(key + " must be a 4x4 matrix");
    }
    for (std::size_t column = 0; column < 4; ++column) {
      result.matrix()(row, column) = node[row][column].as<double>();
    }
  }
  return result;
}

YAML::Node matrixNode(const Eigen::Isometry3d & transform)
{
  YAML::Node matrix;
  for (Eigen::Index row = 0; row < 4; ++row) {
    YAML::Node values;
    for (Eigen::Index column = 0; column < 4; ++column) {
      values.push_back(transform.matrix()(row, column));
    }
    matrix.push_back(values);
  }
  return matrix;
}

YAML::Node vectorNode(const Eigen::Vector3d & vector)
{
  YAML::Node node;
  for (Eigen::Index index = 0; index < 3; ++index) {
    node.push_back(vector(index));
  }
  return node;
}

void writeYaml(const std::string & path, const YAML::Node & root)
{
  const std::filesystem::path output(path);
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  const std::filesystem::path temporary = output.string() + ".tmp";
  {
    std::ofstream stream(temporary);
    if (!stream) {
      throw std::runtime_error("cannot open output YAML: " + temporary.string());
    }
    stream << root;
  }
  std::filesystem::rename(temporary, output);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 5) {
    std::cerr << "Usage: reach_refinement_analyzer RECORDS_YAML OUTPUT_YAML "
              << "[SAMPLE_COUNT] [--translation-only]\n";
    return 2;
  }
  try {
    std::size_t requested_count = 3U;
    bool translation_only = false;
    for (int index = 3; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--translation-only") {
        translation_only = true;
      } else {
        requested_count = static_cast<std::size_t>(std::stoul(argument));
      }
    }
    if (requested_count < 3) {
      throw std::runtime_error("SAMPLE_COUNT must be at least three");
    }
    const YAML::Node database = YAML::LoadFile(argv[1]);
    const YAML::Node records = database["records"];
    if (!records || !records.IsSequence()) {
      throw std::runtime_error("records YAML has no records sequence");
    }

    struct SelectedSample
    {
      std::string record_id;
      std::string acquisition_id;
      std::string source_frame;
      plasma_eye_hand::ReachRefinementSample sample;
    };
    std::vector<SelectedSample> eligible;
    for (const auto & record : records) {
      if (!record["T_base_to_target_nozzle_tip"] ||
        !record["T_base_to_observed_nozzle_tip"] ||
        !record["source_geometry"] ||
        !record["source_geometry"]["T_source_to_target_nozzle_tip"] ||
        !record["source_geometry"]["frame_id"])
      {
        continue;
      }
      SelectedSample selected;
      selected.record_id = record["record_id"].as<std::string>();
      selected.acquisition_id = record["acquisition_id"].as<std::string>();
      selected.source_frame = record["source_geometry"]["frame_id"].as<std::string>();
      selected.sample.base_to_target = matrixFromYaml(
        record["T_base_to_target_nozzle_tip"], "T_base_to_target_nozzle_tip");
      selected.sample.source_to_target = matrixFromYaml(
        record["source_geometry"]["T_source_to_target_nozzle_tip"],
        "T_source_to_target_nozzle_tip");
      selected.sample.base_to_observed = matrixFromYaml(
        record["T_base_to_observed_nozzle_tip"], "T_base_to_observed_nozzle_tip");
      eligible.push_back(selected);
    }
    if (eligible.size() < requested_count) {
      throw std::runtime_error(
              "only " + std::to_string(eligible.size()) +
              " records contain complete source geometry");
    }
    eligible.erase(eligible.begin(), eligible.end() - requested_count);
    const std::string source_frame = eligible.front().source_frame;
    std::vector<plasma_eye_hand::ReachRefinementSample> samples;
    for (const auto & selected : eligible) {
      if (selected.source_frame != source_frame) {
        throw std::runtime_error("selected records use different source frames");
      }
      samples.push_back(selected.sample);
    }

    const auto result = translation_only ?
      plasma_eye_hand::fitSourceTranslationCorrection(samples) :
      plasma_eye_hand::fitSourceCorrection(samples);
    YAML::Node output;
    output["schema_version"] = 1;
    output["name"] = translation_only ?
      "entry_reach_translation_candidate_20260726" :
      "entry_reach_refinement_candidate_20260725";
    output["status"] = "training_candidate_only";
    output["validated"] = false;
    output["execution_allowed"] = false;
    output["motion_commands_published"] = false;
    output["method"] = translation_only ?
      "three_or_more_point_translation_only_source_frame_correction" :
      "three_or_more_point_kabsch_source_frame_correction";
    output["source_frame"] = source_frame;
    output["meaning"] =
      "T_base_source_corrected = T_base_source_current * T_source_correction";
    for (const auto & selected : eligible) {
      output["training_record_ids"].push_back(selected.record_id);
      output["training_acquisition_ids"].push_back(selected.acquisition_id);
    }
    output["T_source_correction"] = matrixNode(result.source_correction);
    const double rotation_deg = std::abs(Eigen::AngleAxisd(
        result.source_correction.linear()).angle()) * 180.0 / M_PI;
    output["metrics"]["rotation_deg"] = rotation_deg;
    output["metrics"]["translation_mm"] = vectorNode(
      result.source_correction.translation() * 1000.0);
    output["metrics"]["singular_values"] = vectorNode(result.singular_values);
    for (const double residual : result.residual_m) {
      output["metrics"]["training_residual_mm"].push_back(residual * 1000.0);
    }
    output["metrics"]["training_rms_mm"] = result.rms_residual_m * 1000.0;
    output["metrics"]["training_max_mm"] = result.max_residual_m * 1000.0;
    output["review"]["independent_blind_validation_required"] = true;
    output["review"]["may_replace_handeye_yaml"] = false;
    output["review"]["may_enable_motion"] = false;
    output["review"]["note"] = translation_only ?
      "Translation-only candidate; preview and independent blind validation are required." :
      "Candidate is fitted to the same entry observations and is not an acceptance result.";
    writeYaml(argv[2], output);

    std::cout << std::fixed << std::setprecision(3)
              << "source correction candidate: rotation=" << rotation_deg
              << " deg, translation=[" << result.source_correction.translation().x() * 1000.0
              << ", " << result.source_correction.translation().y() * 1000.0
              << ", " << result.source_correction.translation().z() * 1000.0
              << "] mm, training RMS=" << result.rms_residual_m * 1000.0
              << " mm, max=" << result.max_residual_m * 1000.0 << " mm\n"
              << "Training-only candidate written to " << argv[2] << '\n';
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "reach refinement analysis failed: " << exception.what() << '\n';
    return 1;
  }
}
