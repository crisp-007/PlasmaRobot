#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Edge = std::pair<std::uint32_t, std::uint32_t>;

struct MeshData
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
};

struct BoundaryFit
{
  std::size_t component_count = 0;
  std::size_t point_count = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

MeshData loadMesh(const std::string & path)
{
  MeshData data;
  if (pcl::io::loadPLYFile(path, data.mesh) != 0) {
    throw std::runtime_error("failed to read " + path);
  }
  pcl::fromPCLPointCloud2(data.mesh.cloud, data.cloud);
  data.points.reserve(data.cloud.size());
  for (const auto & point : data.cloud) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const Eigen::Vector3d value(point.x, point.y, point.z);
    data.points.push_back(value);
    data.center += value;
  }
  if (data.points.size() < 3) {
    throw std::runtime_error("mesh has too few finite points: " + path);
  }
  data.center /= static_cast<double>(data.points.size());
  return data;
}

Eigen::Matrix3d covarianceOf(
  const std::vector<Eigen::Vector3d> & points, const Eigen::Vector3d & center)
{
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : points) {
    const Eigen::Vector3d delta = point - center;
    covariance += delta * delta.transpose();
  }
  return covariance / static_cast<double>(points.size() - 1);
}

Eigen::Vector3d smallestPcaAxis(const std::vector<Eigen::Vector3d> & points)
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto & point : points) {
    center += point;
  }
  center /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covarianceOf(points, center));
  return solver.eigenvectors().col(0).normalized();
}

BoundaryFit largestBoundary(const MeshData & data)
{
  std::map<Edge, int> edge_counts;
  for (const auto & polygon : data.mesh.polygons) {
    for (std::size_t index = 0; index < polygon.vertices.size(); ++index) {
      std::uint32_t first = polygon.vertices[index];
      std::uint32_t second = polygon.vertices[(index + 1) % polygon.vertices.size()];
      if (first > second) {
        std::swap(first, second);
      }
      ++edge_counts[{first, second}];
    }
  }
  std::map<std::uint32_t, std::vector<std::uint32_t>> adjacency;
  for (const auto & entry : edge_counts) {
    if (entry.second != 1) {
      continue;
    }
    adjacency[entry.first.first].push_back(entry.first.second);
    adjacency[entry.first.second].push_back(entry.first.first);
  }

  std::set<std::uint32_t> visited;
  std::vector<std::uint32_t> largest;
  BoundaryFit fit;
  for (const auto & entry : adjacency) {
    if (visited.count(entry.first) != 0) {
      continue;
    }
    ++fit.component_count;
    std::vector<std::uint32_t> component;
    std::deque<std::uint32_t> queue{entry.first};
    visited.insert(entry.first);
    while (!queue.empty()) {
      const std::uint32_t current = queue.front();
      queue.pop_front();
      component.push_back(current);
      for (const std::uint32_t neighbor : adjacency[current]) {
        if (visited.insert(neighbor).second) {
          queue.push_back(neighbor);
        }
      }
    }
    if (component.size() > largest.size()) {
      largest = std::move(component);
    }
  }

  std::vector<Eigen::Vector3d> points;
  for (const std::uint32_t index : largest) {
    if (index >= data.cloud.size() || !pcl::isFinite(data.cloud[index])) {
      continue;
    }
    points.emplace_back(data.cloud[index].x, data.cloud[index].y, data.cloud[index].z);
    fit.center += points.back();
  }
  if (points.size() < 3) {
    throw std::runtime_error("mesh has no usable boundary component");
  }
  fit.point_count = points.size();
  fit.center /= static_cast<double>(points.size());
  fit.normal = smallestPcaAxis(points);
  if (fit.normal.dot(data.center - fit.center) < 0.0) {
    fit.normal = -fit.normal;
  }
  return fit;
}

Eigen::Isometry3d matrixFromYaml(const YAML::Node & node, const std::string & key)
{
  if (!node || !node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(key + " must be a 4x4 matrix");
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      transform.matrix()(row, column) = node[row][column].as<double>();
    }
  }
  return transform;
}

double angleDeg(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const double dot = std::clamp(first.normalized().dot(second.normalized()), -1.0, 1.0);
  return std::acos(dot) * 180.0 / M_PI;
}

Eigen::Vector3d orientedLike(
  const Eigen::Vector3d & candidate, const Eigen::Vector3d & reference)
{
  return candidate.dot(reference) < 0.0 ? -candidate : candidate;
}

void printCandidate(
  const std::string & name, const Eigen::Vector3d & axis,
  const Eigen::Vector3d & actual_source_axis)
{
  std::cout << name << '=' << axis.transpose()
            << " angle_to_actual_deg=" << angleDeg(axis, actual_source_axis) << '\n';
}

Eigen::Vector3d deepestAnchor(
  const MeshData & mesh, const Eigen::Vector3d & origin,
  const Eigen::Vector3d & inward_axis, std::size_t * point_count)
{
  std::vector<double> depths;
  depths.reserve(mesh.points.size());
  for (const auto & point : mesh.points) {
    depths.push_back((point - origin).dot(inward_axis));
  }
  std::vector<double> sorted = depths;
  std::sort(sorted.begin(), sorted.end());
  const std::size_t threshold_index = static_cast<std::size_t>(
    std::floor(0.90 * static_cast<double>(sorted.size() - 1)));
  const double threshold = sorted[threshold_index];
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  *point_count = 0;
  for (std::size_t index = 0; index < mesh.points.size(); ++index) {
    if (depths[index] < threshold) {
      continue;
    }
    center += mesh.points[index];
    ++(*point_count);
  }
  if (*point_count < 3) {
    throw std::runtime_error("too few points in deepest anchor");
  }
  return center / static_cast<double>(*point_count);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4 && argc != 5) {
    std::cerr << "usage: entry_axis_candidate_diagnostic FULL_MESH SURGICAL_MESH "
              << "BLIND_RECORD_YAML [SECOND_SURGICAL_MESH]\n";
    return 2;
  }
  try {
    const MeshData full = loadMesh(argv[1]);
    const MeshData surgical = loadMesh(argv[2]);
    const BoundaryFit full_boundary = largestBoundary(full);
    const BoundaryFit surgical_boundary = largestBoundary(surgical);

    const YAML::Node root = YAML::LoadFile(argv[3]);
    const YAML::Node records = root["records"];
    if (!records || !records.IsSequence() || records.size() == 0) {
      throw std::runtime_error("blind record YAML has no records");
    }
    const YAML::Node record = records[records.size() - 1];
    const Eigen::Isometry3d base_to_target = matrixFromYaml(
      record["T_base_to_target_nozzle_tip"], "T_base_to_target_nozzle_tip");
    const Eigen::Isometry3d source_to_target = matrixFromYaml(
      record["source_geometry"]["T_source_to_target_nozzle_tip"],
      "T_source_to_target_nozzle_tip");
    const Eigen::Isometry3d base_to_observed = matrixFromYaml(
      record["T_base_to_observed_nozzle_tip"], "T_base_to_observed_nozzle_tip");
    const Eigen::Isometry3d base_to_source = base_to_target * source_to_target.inverse();
    const Eigen::Matrix3d base_to_source_rotation = base_to_source.linear();
    const Eigen::Vector3d target_source_axis = source_to_target.linear().col(0).normalized();
    const Eigen::Vector3d actual_source_axis =
      (base_to_source_rotation.transpose() * base_to_observed.linear().col(0)).normalized();

    Eigen::Vector3d full_pca = smallestPcaAxis(full.points);
    full_pca = orientedLike(full_pca, target_source_axis);
    Eigen::Vector3d surgical_pca = smallestPcaAxis(surgical.points);
    surgical_pca = orientedLike(surgical_pca, target_source_axis);
    const Eigen::Vector3d full_boundary_axis =
      orientedLike(full_boundary.normal, target_source_axis);
    const Eigen::Vector3d surgical_boundary_axis =
      orientedLike(surgical_boundary.normal, target_source_axis);

    std::cout << std::fixed << std::setprecision(9);
    std::cout << "target_source_axis=" << target_source_axis.transpose() << '\n';
    std::cout << "actual_source_axis=" << actual_source_axis.transpose() << '\n';
    std::cout << "recorded_axis_error_deg="
              << angleDeg(target_source_axis, actual_source_axis) << '\n';
    printCandidate("full_boundary_axis", full_boundary_axis, actual_source_axis);
    printCandidate("full_mesh_pca_axis", full_pca, actual_source_axis);
    printCandidate("surgical_boundary_axis", surgical_boundary_axis, actual_source_axis);
    printCandidate("surgical_mesh_pca_axis", surgical_pca, actual_source_axis);
    std::cout << "full_boundary_components=" << full_boundary.component_count
              << " points=" << full_boundary.point_count
              << " center=" << full_boundary.center.transpose() << '\n';
    std::cout << "surgical_boundary_components=" << surgical_boundary.component_count
              << " points=" << surgical_boundary.point_count
              << " center=" << surgical_boundary.center.transpose() << '\n';
    Eigen::Isometry3d corrected_source_target = source_to_target;
    corrected_source_target.translation() = full_boundary.center;
    const Eigen::Isometry3d corrected_base_target =
      base_to_source * corrected_source_target;
    const Eigen::Vector3d corrected_delta =
      base_to_observed.translation() - corrected_base_target.translation();
    const Eigen::Vector3d corrected_outward_axis =
      -(base_to_source_rotation * full_boundary_axis).normalized();
    const double corrected_axial = corrected_delta.dot(corrected_outward_axis);
    const Eigen::Vector3d corrected_lateral =
      corrected_delta - corrected_axial * corrected_outward_axis;
    std::cout << "corrected_mouth_target_base="
              << corrected_base_target.translation().transpose() << '\n';
    std::cout << "corrected_observed_minus_target=" << corrected_delta.transpose()
              << " norm_mm=" << corrected_delta.norm() * 1000.0
              << " axial_mm=" << corrected_axial * 1000.0
              << " lateral_mm=" << corrected_lateral.norm() * 1000.0 << '\n';
    if (argc == 5) {
      const MeshData second_surgical = loadMesh(argv[4]);
      const BoundaryFit second_boundary = largestBoundary(second_surgical);
      std::size_t first_anchor_count = 0;
      std::size_t second_anchor_count = 0;
      const Eigen::Vector3d first_anchor = deepestAnchor(
        surgical, full_boundary.center, full_boundary_axis, &first_anchor_count);
      const Eigen::Vector3d second_anchor = deepestAnchor(
        second_surgical, full_boundary.center, full_boundary_axis, &second_anchor_count);
      const Eigen::Vector3d anchor_delta = second_anchor - first_anchor;
      const double axial_delta = anchor_delta.dot(full_boundary_axis);
      const Eigen::Vector3d transverse_delta =
        anchor_delta - axial_delta * full_boundary_axis;
      std::cout << "first_deep_anchor=" << first_anchor.transpose()
                << " points=" << first_anchor_count << '\n';
      std::cout << "second_deep_anchor=" << second_anchor.transpose()
                << " points=" << second_anchor_count << '\n';
      std::cout << "anchor_delta=" << anchor_delta.transpose()
                << " norm_mm=" << anchor_delta.norm() * 1000.0
                << " axial_mm=" << axial_delta * 1000.0
                << " transverse_mm=" << transverse_delta.norm() * 1000.0 << '\n';
      std::cout << "fixed_axis_direction_drift_deg=0.000000000\n";
      std::cout << "second_surgical_boundary_center="
                << second_boundary.center.transpose()
                << " components=" << second_boundary.component_count
                << " points=" << second_boundary.point_count << '\n';
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "entry axis diagnostic failed: " << error.what() << '\n';
    return 1;
  }
}
