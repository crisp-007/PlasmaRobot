#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

using Edge = std::pair<std::uint32_t, std::uint32_t>;

Eigen::Vector3d pointVector(const pcl::PointXYZ &point)
{
    return {point.x, point.y, point.z};
}

void printPca(const std::string &label,
              const std::vector<Eigen::Vector3d> &points)
{
    if (points.size() < 3) {
        std::cout << label << ": too few points\n";
        return;
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto &point : points)
        center += point;
    center /= static_cast<double>(points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : points) {
        const Eigen::Vector3d delta = point - center;
        covariance += delta * delta.transpose();
    }
    covariance /= static_cast<double>(points.size() - 1);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    std::cout << label << ": count=" << points.size()
              << " center=" << center.transpose() << '\n';
    std::cout << "  eigenvalues=" << solver.eigenvalues().transpose() << '\n';
    for (int index = 0; index < 3; ++index)
        std::cout << "  axis[" << index << "]="
                  << solver.eigenvectors().col(index).transpose() << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: mesh_axis_diagnostic <mesh.ply>\n";
        return 2;
    }

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPLYFile(argv[1], mesh) != 0) {
        std::cerr << "failed to read " << argv[1] << '\n';
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    std::vector<Eigen::Vector3d> allPoints;
    allPoints.reserve(cloud.size());
    for (const auto &point : cloud)
        if (pcl::isFinite(point))
            allPoints.push_back(pointVector(point));

    std::cout << std::fixed << std::setprecision(9)
              << "vertices=" << cloud.size()
              << " faces=" << mesh.polygons.size() << '\n';
    printPca("mesh", allPoints);

    std::map<Edge, int> edgeCounts;
    std::map<std::uint32_t, std::vector<std::uint32_t>> boundaryAdjacency;
    for (const auto &polygon : mesh.polygons) {
        const auto &vertices = polygon.vertices;
        for (std::size_t index = 0; index < vertices.size(); ++index) {
            std::uint32_t first = vertices[index];
            std::uint32_t second = vertices[(index + 1) % vertices.size()];
            if (first > second)
                std::swap(first, second);
            ++edgeCounts[{first, second}];
        }
    }
    for (const auto &[edge, count] : edgeCounts) {
        if (count != 1)
            continue;
        boundaryAdjacency[edge.first].push_back(edge.second);
        boundaryAdjacency[edge.second].push_back(edge.first);
    }

    std::set<std::uint32_t> visited;
    std::vector<std::vector<std::uint32_t>> components;
    for (const auto &[start, neighbors] : boundaryAdjacency) {
        (void)neighbors;
        if (visited.count(start) != 0)
            continue;
        std::deque<std::uint32_t> queue{start};
        visited.insert(start);
        components.emplace_back();
        while (!queue.empty()) {
            const std::uint32_t current = queue.front();
            queue.pop_front();
            components.back().push_back(current);
            for (const std::uint32_t neighbor : boundaryAdjacency[current]) {
                if (visited.insert(neighbor).second)
                    queue.push_back(neighbor);
            }
        }
    }
    std::sort(components.begin(), components.end(),
              [](const auto &left, const auto &right) {
                  return left.size() > right.size();
              });
    std::cout << "boundary_components=" << components.size() << '\n';
    for (std::size_t componentIndex = 0;
         componentIndex < std::min<std::size_t>(components.size(), 10);
         ++componentIndex) {
        std::vector<Eigen::Vector3d> points;
        for (const std::uint32_t pointIndex : components[componentIndex]) {
            if (pointIndex < cloud.size() && pcl::isFinite(cloud[pointIndex]))
                points.push_back(pointVector(cloud[pointIndex]));
        }
        printPca("boundary[" + std::to_string(componentIndex) + "]", points);
    }

    if (!components.empty()) {
        std::vector<Eigen::Vector3d> boundaryPoints;
        for (const std::uint32_t pointIndex : components.front()) {
            if (pointIndex < cloud.size() && pcl::isFinite(cloud[pointIndex]))
                boundaryPoints.push_back(pointVector(cloud[pointIndex]));
        }
        Eigen::Vector3d meshCenter = Eigen::Vector3d::Zero();
        for (const auto &point : allPoints)
            meshCenter += point;
        meshCenter /= static_cast<double>(allPoints.size());
        Eigen::Vector3d boundaryCenter = Eigen::Vector3d::Zero();
        for (const auto &point : boundaryPoints)
            boundaryCenter += point;
        boundaryCenter /= static_cast<double>(boundaryPoints.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &point : boundaryPoints) {
            const Eigen::Vector3d delta = point - boundaryCenter;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(boundaryPoints.size() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        Eigen::Vector3d inward = solver.eigenvectors().col(0).normalized();
        if (inward.dot(meshCenter - boundaryCenter) < 0.0)
            inward = -inward;

        std::vector<std::pair<double, Eigen::Vector3d>> depths;
        depths.reserve(allPoints.size());
        for (const auto &point : allPoints)
            depths.push_back({inward.dot(point - boundaryCenter), point});
        std::sort(depths.begin(), depths.end(),
                  [](const auto &left, const auto &right) {
                      return left.first < right.first;
                  });
        std::cout << "inward_axis=" << inward.transpose() << '\n';
        for (const double fraction : {0.10, 0.05, 0.02, 0.01}) {
            const std::size_t count = std::max<std::size_t>(
                1, static_cast<std::size_t>(depths.size() * fraction));
            Eigen::Vector3d deepCenter = Eigen::Vector3d::Zero();
            for (std::size_t index = depths.size() - count;
                 index < depths.size(); ++index) {
                deepCenter += depths[index].second;
            }
            deepCenter /= static_cast<double>(count);
            const double axial = inward.dot(deepCenter - boundaryCenter);
            const Eigen::Vector3d entryCenter = deepCenter - axial * inward;
            std::cout << "deepest_" << fraction * 100.0 << "pct: depth="
                      << axial << " deep_center=" << deepCenter.transpose()
                      << " entry_center=" << entryCenter.transpose() << '\n';
        }
    }
    return 0;
}
