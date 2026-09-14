#include "point_deal.h"
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QRegularExpression>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkCell.h>
#include <vtkCellData.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkCutter.h>
#include <vtkDoubleArray.h>
#include <vtkFieldData.h>
#include <vtkLine.h>
#include <vtkIntArray.h>
#include <vtkMath.h>
#include <vtkPlane.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricSpline.h>
#include <vtkStripper.h>
#include <vtkTriangle.h>
#include <vtkUnsignedCharArray.h>
#include <QMetaObject>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <unordered_map>

namespace {
using RebuildPointT = pcl::PointXYZRGB;
using RebuildPointNormalT = pcl::PointXYZRGBNormal;

// Keep the unconstrained MoveIt segment outside the patient-side entry region.
constexpr double kEntryNormalApproachDistanceM = 0.100;

// The fitted contour and sampled path can differ by a few hundredths of a
// millimeter. Keep the configured clearance unchanged and tolerate only this
// sub-voxel numerical error when comparing the two values.
constexpr double kClearanceComparisonToleranceM = 0.00005;

bool isFinitePoint(const RebuildPointT &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool isFiniteNormal(const pcl::Normal &n)
{
    return std::isfinite(n.normal_x) &&
           std::isfinite(n.normal_y) &&
           std::isfinite(n.normal_z);
}

struct AxisVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

AxisVec3 makeVec(double x, double y, double z)
{
    return {x, y, z};
}

AxisVec3 addVec(const AxisVec3 &a, const AxisVec3 &b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

AxisVec3 subVec(const AxisVec3 &a, const AxisVec3 &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

AxisVec3 mulVec(const AxisVec3 &a, double s)
{
    return {a.x * s, a.y * s, a.z * s};
}

double dotVec(const AxisVec3 &a, const AxisVec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

AxisVec3 crossVec(const AxisVec3 &a, const AxisVec3 &b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double normVec(const AxisVec3 &a)
{
    return std::sqrt(dotVec(a, a));
}

AxisVec3 normalizedVec(const AxisVec3 &a)
{
    const double n = normVec(a);
    if (n <= 1e-12)
        return {};
    return mulVec(a, 1.0 / n);
}

AxisVec3 meanVec(const std::vector<AxisVec3> &points)
{
    AxisVec3 mean;
    if (points.empty())
        return mean;
    for (const auto &p : points)
        mean = addVec(mean, p);
    return mulVec(mean, 1.0 / points.size());
}

AxisVec3 pcaDirection(const std::vector<AxisVec3> &points, bool largestEigen)
{
    if (points.size() < 3)
        return {};

    const AxisVec3 mean = meanVec(points);
    double cov[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    for (const auto &p : points) {
        const AxisVec3 d = subVec(p, mean);
        const double v[3] = {d.x, d.y, d.z};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                cov[r][c] += v[r] * v[c] / std::max(1.0, static_cast<double>(points.size() - 1));
    }

    double eigVals[3] = {0.0, 0.0, 0.0};
    double eigVecs[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double *covRows[3] = {cov[0], cov[1], cov[2]};
    double *eigRows[3] = {eigVecs[0], eigVecs[1], eigVecs[2]};
    vtkMath::Jacobi(covRows, eigVals, eigRows);

    int idx = 0;
    for (int i = 1; i < 3; ++i) {
        if ((largestEigen && eigVals[i] > eigVals[idx]) ||
            (!largestEigen && eigVals[i] < eigVals[idx])) {
            idx = i;
        }
    }

    return normalizedVec(makeVec(eigVecs[0][idx], eigVecs[1][idx], eigVecs[2][idx]));
}

vtkSmartPointer<vtkPolyData> pointsToVertices(const std::vector<AxisVec3> &points)
{
    vtkSmartPointer<vtkPoints> vtkPts = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
    vtkPts->Allocate(static_cast<vtkIdType>(points.size()));
    verts->AllocateEstimate(static_cast<vtkIdType>(points.size()), 1);
    for (const auto &p : points) {
        vtkIdType id = vtkPts->InsertNextPoint(p.x, p.y, p.z);
        verts->InsertNextCell(1, &id);
    }

    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(vtkPts);
    poly->SetVerts(verts);
    return poly;
}

vtkSmartPointer<vtkPolyData> pointsToPolyline(const std::vector<AxisVec3> &points)
{
    vtkSmartPointer<vtkPoints> vtkPts = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkPts->Allocate(static_cast<vtkIdType>(points.size()));
    for (const auto &p : points)
        vtkPts->InsertNextPoint(p.x, p.y, p.z);

    if (points.size() >= 2) {
        lines->InsertNextCell(static_cast<vtkIdType>(points.size()));
        for (vtkIdType i = 0; i < static_cast<vtkIdType>(points.size()); ++i)
            lines->InsertCellPoint(i);
    }

    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(vtkPts);
    poly->SetLines(lines);
    return poly;
}

double wrapTo360(double angle)
{
    angle = std::fmod(angle, 360.0);
    return angle < 0.0 ? angle + 360.0 : angle;
}

double wrapTo180(double angle)
{
    angle = wrapTo360(angle + 180.0) - 180.0;
    return angle;
}

double pchipEndpointSlope(double h0, double h1, double delta0, double delta1)
{
    double slope = ((2.0 * h0 + h1) * delta0 - h0 * delta1) / (h0 + h1);
    if (slope * delta0 <= 0.0)
        return 0.0;
    if (delta0 * delta1 < 0.0 && std::abs(slope) > 3.0 * std::abs(delta0))
        return 3.0 * delta0;
    return slope;
}

std::vector<double> pchipSlopes(const std::vector<double> &x, const std::vector<double> &y)
{
    const std::size_t count = x.size();
    std::vector<double> slopes(count, 0.0);
    if (count < 2)
        return slopes;

    std::vector<double> h(count - 1, 0.0);
    std::vector<double> delta(count - 1, 0.0);
    for (std::size_t i = 0; i + 1 < count; ++i) {
        h[i] = x[i + 1] - x[i];
        delta[i] = (y[i + 1] - y[i]) / h[i];
    }
    if (count == 2) {
        slopes[0] = delta[0];
        slopes[1] = delta[0];
        return slopes;
    }

    slopes[0] = pchipEndpointSlope(h[0], h[1], delta[0], delta[1]);
    slopes[count - 1] = pchipEndpointSlope(h[count - 2], h[count - 3],
                                           delta[count - 2], delta[count - 3]);
    for (std::size_t i = 1; i + 1 < count; ++i) {
        if (delta[i - 1] * delta[i] <= 0.0) {
            slopes[i] = 0.0;
            continue;
        }
        const double w1 = 2.0 * h[i] + h[i - 1];
        const double w2 = h[i] + 2.0 * h[i - 1];
        slopes[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
    }
    return slopes;
}

double pchipInterpolate(const std::vector<double> &x,
                        const std::vector<double> &y,
                        const std::vector<double> &slopes,
                        double query)
{
    if (query <= x.front())
        return y.front();
    if (query >= x.back())
        return y.back();

    auto upper = std::upper_bound(x.begin(), x.end(), query);
    const std::size_t i = static_cast<std::size_t>(std::distance(x.begin(), upper) - 1);
    const double h = x[i + 1] - x[i];
    const double t = (query - x[i]) / h;
    const double t2 = t * t;
    const double t3 = t2 * t;
    return (2.0 * t3 - 3.0 * t2 + 1.0) * y[i]
         + (t3 - 2.0 * t2 + t) * h * slopes[i]
         + (-2.0 * t3 + 3.0 * t2) * y[i + 1]
         + (t3 - t2) * h * slopes[i + 1];
}

std::array<unsigned char, 3> matlabLinesColor(int regionId)
{
    static const std::array<std::array<unsigned char, 3>, 7> colors = {{
        {{0, 114, 189}}, {{217, 83, 25}}, {{237, 177, 32}},
        {{126, 47, 142}}, {{119, 172, 48}}, {{77, 190, 238}},
        {{162, 20, 47}}
    }};
    if (regionId <= 0)
        return {{51, 191, 242}};
    return colors[static_cast<std::size_t>((regionId - 1) % colors.size())];
}
}

// ==========================================
// PathPlanningTask 实现
// ==========================================

PathPlanningTask::PathPlanningTask(const QString &sourceName,
                                   vtkSmartPointer<vtkPolyData> surgicalMesh,
                                   vtkSmartPointer<vtkPolyData> referenceMesh,
                                   int sampleCount,
                                   const QString &rodType,
                                   double voxelSize,
                                   double medialPercentile,
                                   int sectionCount,
                                   double sectionHalfWidth,
                                   int smoothPointsNum,
                                   int smoothWindow,
                                   double excludeOpeningDistance,
                                   int excludeOpeningLayers,
                                   double outletRearExtent,
                                   double safetyForwardExtent,
                                   double safeMarginBottom,
                                   double safeMarginTop,
                                   QObject *receiver)
    : m_sourceName(sourceName),
      m_surgicalMesh(surgicalMesh),
      m_referenceMesh(referenceMesh),
      m_sampleCount(std::max(100, sampleCount)),
      m_rodType(rodType),
      m_voxelSize(std::clamp(voxelSize, 0.0001, 0.02)),
      m_medialPercentile(std::clamp(medialPercentile, 50.0, 95.0)),
      m_sectionCount(std::clamp(sectionCount, 10, 200)),
      m_sectionHalfWidth(std::max(0.0, sectionHalfWidth)),
      m_smoothPointsNum(std::clamp(smoothPointsNum, 20, 400)),
      m_smoothWindow(std::clamp(smoothWindow | 1, 1, 51)),
      m_excludeOpeningDistance(std::max(0.0, excludeOpeningDistance)),
      m_excludeOpeningLayers(std::clamp(excludeOpeningLayers, 0, 30)),
      m_outletRearExtent(std::clamp(outletRearExtent, 0.0, 0.2)),
      m_safetyForwardExtent(std::clamp(safetyForwardExtent, 0.0, 0.2)),
      m_safeMarginBottom(std::max(0.0, safeMarginBottom)),
      m_safeMarginTop(std::max(0.0, safeMarginTop)),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void PathPlanningTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onPathPlanningProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName),
                              Q_ARG(int, progress));
}

void PathPlanningTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onPathPlanningLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName),
                              Q_ARG(QString, message));
}

void PathPlanningTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> sampledCloud,
                         vtkSmartPointer<vtkPolyData> curvedAxis,
                         vtkSmartPointer<vtkPolyData> straightAxis,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onPathPlanningFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, sampledCloud),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, curvedAxis),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, straightAxis),
                                  Q_ARG(bool, ok),
                                  Q_ARG(QString, message));
    };

    try {
        if (!m_surgicalMesh || m_surgicalMesh->GetNumberOfPoints() < 3 || m_surgicalMesh->GetNumberOfCells() == 0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("术区 mesh 为空，无法路径规划"));
            return;
        }

        struct TriInfo {
            AxisVec3 p0;
            AxisVec3 p1;
            AxisVec3 p2;
            double area = 0.0;
        };

        std::vector<AxisVec3> vertices;
        vertices.reserve(static_cast<std::size_t>(m_surgicalMesh->GetNumberOfPoints()));
        AxisVec3 meshCenter;
        AxisVec3 vmin = makeVec(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        AxisVec3 vmax = makeVec(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
        for (vtkIdType i = 0; i < m_surgicalMesh->GetNumberOfPoints(); ++i) {
            double p[3];
            m_surgicalMesh->GetPoint(i, p);
            AxisVec3 v = makeVec(p[0], p[1], p[2]);
            vertices.push_back(v);
            meshCenter = addVec(meshCenter, v);
            vmin.x = std::min(vmin.x, v.x); vmin.y = std::min(vmin.y, v.y); vmin.z = std::min(vmin.z, v.z);
            vmax.x = std::max(vmax.x, v.x); vmax.y = std::max(vmax.y, v.y); vmax.z = std::max(vmax.z, v.z);
        }
        meshCenter = mulVec(meshCenter, 1.0 / std::max<std::size_t>(1, vertices.size()));

        std::vector<TriInfo> triangles;
        triangles.reserve(static_cast<std::size_t>(m_surgicalMesh->GetNumberOfCells()));
        double totalArea = 0.0;
        struct EdgeKey {
            vtkIdType a;
            vtkIdType b;
            bool operator==(const EdgeKey &o) const { return a == o.a && b == o.b; }
        };
        struct EdgeHash {
            std::size_t operator()(const EdgeKey &e) const
            {
                return std::hash<long long>()((static_cast<long long>(e.a) << 32) ^ static_cast<long long>(e.b));
            }
        };
        std::unordered_map<EdgeKey, int, EdgeHash> edgeCount;

        auto addEdge = [&edgeCount](vtkIdType a, vtkIdType b) {
            if (a > b)
                std::swap(a, b);
            edgeCount[{a, b}]++;
        };

        for (vtkIdType cellId = 0; cellId < m_surgicalMesh->GetNumberOfCells(); ++cellId) {
            vtkCell *cell = m_surgicalMesh->GetCell(cellId);
            if (!cell || cell->GetNumberOfPoints() < 3)
                continue;

            const vtkIdType npts = cell->GetNumberOfPoints();
            for (vtkIdType i = 0; i < npts; ++i)
                addEdge(cell->GetPointId(i), cell->GetPointId((i + 1) % npts));

            const AxisVec3 base = vertices[static_cast<std::size_t>(cell->GetPointId(0))];
            for (vtkIdType k = 1; k + 1 < npts; ++k) {
                TriInfo tri;
                tri.p0 = base;
                tri.p1 = vertices[static_cast<std::size_t>(cell->GetPointId(k))];
                tri.p2 = vertices[static_cast<std::size_t>(cell->GetPointId(k + 1))];
                double a[3] = {tri.p0.x, tri.p0.y, tri.p0.z};
                double b[3] = {tri.p1.x, tri.p1.y, tri.p1.z};
                double c[3] = {tri.p2.x, tri.p2.y, tri.p2.z};
                tri.area = vtkTriangle::TriangleArea(a, b, c);
                if (tri.area > 1e-12 && std::isfinite(tri.area)) {
                    totalArea += tri.area;
                    triangles.push_back(tri);
                }
            }
        }
        if (triangles.empty() || totalArea <= 0.0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("术区 mesh 没有有效三角面"));
            return;
        }

        reportProgress(2);
        reportLog(QStringLiteral("算法参数：采样点=%1，喷杆=%2，体素=%3，中心候选百分位=%4，分层=%5，截面半宽=%6，平滑点数=%7，平滑窗口=%8，开口距离剔除=%9，开口层数剔除=%10，喷口后向占用=%11，安全点前向占用=%12，底部额外余量=%13，开口额外余量=%14")
            .arg(m_sampleCount).arg(m_rodType).arg(m_voxelSize).arg(m_medialPercentile)
            .arg(m_sectionCount).arg(m_sectionHalfWidth).arg(m_smoothPointsNum).arg(m_smoothWindow)
            .arg(m_excludeOpeningDistance).arg(m_excludeOpeningLayers)
            .arg(m_outletRearExtent).arg(m_safetyForwardExtent)
            .arg(m_safeMarginBottom).arg(m_safeMarginTop));

        std::vector<double> cumulative;
        cumulative.reserve(triangles.size());
        double acc = 0.0;
        for (const auto &tri : triangles) {
            acc += tri.area;
            cumulative.push_back(acc);
        }

        std::mt19937 rng(42);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        std::uniform_real_distribution<double> areaPick(0.0, totalArea);
        auto sampleTriangle = [&](const TriInfo &tri) {
            double r1 = unit(rng);
            double r2 = unit(rng);
            if (r1 + r2 > 1.0) {
                r1 = 1.0 - r1;
                r2 = 1.0 - r2;
            }
            return addVec(tri.p0, addVec(mulVec(subVec(tri.p1, tri.p0), r1), mulVec(subVec(tri.p2, tri.p0), r2)));
        };

        std::vector<AxisVec3> sampledSurface;
        sampledSurface.reserve(static_cast<std::size_t>(m_sampleCount));
        for (int i = 0; i < m_sampleCount; ++i) {
            const double pick = areaPick(rng);
            auto it = std::lower_bound(cumulative.begin(), cumulative.end(), pick);
            const auto &tri = triangles[std::min<std::size_t>(std::distance(cumulative.begin(), it), triangles.size() - 1)];
            sampledSurface.push_back(sampleTriangle(tri));
            if (i % std::max(1, m_sampleCount / 20) == 0)
                reportProgress(3 + static_cast<int>(12.0 * i / m_sampleCount));
        }
        vtkSmartPointer<vtkPolyData> sampledCloud = pointsToVertices(sampledSurface);

        double voxelSize = m_voxelSize;
        const double pad = 3.0 * voxelSize;
        auto computeDims = [&]() {
            std::array<int, 3> dims = {
                std::max(8, static_cast<int>(std::ceil((vmax.x - vmin.x + 2 * pad) / voxelSize)) + 1),
                std::max(8, static_cast<int>(std::ceil((vmax.y - vmin.y + 2 * pad) / voxelSize)) + 1),
                std::max(8, static_cast<int>(std::ceil((vmax.z - vmin.z + 2 * pad) / voxelSize)) + 1)
            };
            return dims;
        };
        std::array<int, 3> dims = computeDims();
        std::size_t voxelCount = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
        constexpr std::size_t kMaxVoxels = 2000000;
        if (voxelCount > kMaxVoxels) {
            const double scale = std::cbrt(static_cast<double>(voxelCount) / kMaxVoxels);
            voxelSize *= scale;
            dims = computeDims();
            voxelCount = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
            reportLog(QStringLiteral("体素数量过大，自动调整体素尺寸为 %1，网格=%2x%3x%4")
                .arg(voxelSize).arg(dims[0]).arg(dims[1]).arg(dims[2]));
        }

        AxisVec3 origin = makeVec(vmin.x - 3.0 * voxelSize, vmin.y - 3.0 * voxelSize, vmin.z - 3.0 * voxelSize);
        auto indexOf = [&](int x, int y, int z) {
            return (static_cast<std::size_t>(z) * dims[1] + y) * dims[0] + x;
        };
        auto validVoxel = [&](int x, int y, int z) {
            return x >= 0 && y >= 0 && z >= 0 && x < dims[0] && y < dims[1] && z < dims[2];
        };
        auto worldToVoxel = [&](const AxisVec3 &p, int &x, int &y, int &z) {
            x = static_cast<int>(std::floor((p.x - origin.x) / voxelSize));
            y = static_cast<int>(std::floor((p.y - origin.y) / voxelSize));
            z = static_cast<int>(std::floor((p.z - origin.z) / voxelSize));
            return validVoxel(x, y, z);
        };
        auto voxelToWorld = [&](int x, int y, int z) {
            return makeVec(origin.x + (x + 0.5) * voxelSize,
                           origin.y + (y + 0.5) * voxelSize,
                           origin.z + (z + 0.5) * voxelSize);
        };

        std::vector<unsigned char> occ(voxelCount, 0);
        auto markOcc = [&](const AxisVec3 &p) {
            int x, y, z;
            if (worldToVoxel(p, x, y, z))
                occ[indexOf(x, y, z)] = 1;
        };

        reportProgress(16);
        reportLog(QStringLiteral("[1/8] 表面体素化，网格=%1x%2x%3").arg(dims[0]).arg(dims[1]).arg(dims[2]));
        for (std::size_t ti = 0; ti < triangles.size(); ++ti) {
            const auto &tri = triangles[ti];
            int nSamp = std::max(30, static_cast<int>(std::ceil(tri.area / (0.35 * voxelSize * voxelSize))));
            nSamp = std::min(nSamp, 2500);
            markOcc(tri.p0); markOcc(tri.p1); markOcc(tri.p2);
            for (int s = 0; s < nSamp; ++s)
                markOcc(sampleTriangle(tri));
            if (ti % std::max<std::size_t>(1, triangles.size() / 10) == 0)
                reportProgress(16 + static_cast<int>(14.0 * ti / triangles.size()));
        }

        std::vector<unsigned char> dilated = occ;
        for (int z = 0; z < dims[2]; ++z) {
            for (int y = 0; y < dims[1]; ++y) {
                for (int x = 0; x < dims[0]; ++x) {
                    if (!occ[indexOf(x, y, z)])
                        continue;
                    for (int dz = -1; dz <= 1; ++dz)
                        for (int dy = -1; dy <= 1; ++dy)
                            for (int dx = -1; dx <= 1; ++dx)
                                if (validVoxel(x + dx, y + dy, z + dz))
                                    dilated[indexOf(x + dx, y + dy, z + dz)] = 1;
                }
            }
        }
        occ.swap(dilated);

        reportProgress(32);
        reportLog(QStringLiteral("[2/8] 填充内部体素"));
        std::vector<unsigned char> outside(voxelCount, 0);
        std::deque<std::array<int, 3>> q;
        auto pushOutside = [&](int x, int y, int z) {
            if (!validVoxel(x, y, z))
                return;
            const auto idx = indexOf(x, y, z);
            if (occ[idx] || outside[idx])
                return;
            outside[idx] = 1;
            q.push_back({x, y, z});
        };
        for (int x = 0; x < dims[0]; ++x)
            for (int y = 0; y < dims[1]; ++y) {
                pushOutside(x, y, 0);
                pushOutside(x, y, dims[2] - 1);
            }
        for (int x = 0; x < dims[0]; ++x)
            for (int z = 0; z < dims[2]; ++z) {
                pushOutside(x, 0, z);
                pushOutside(x, dims[1] - 1, z);
            }
        for (int y = 0; y < dims[1]; ++y)
            for (int z = 0; z < dims[2]; ++z) {
                pushOutside(0, y, z);
                pushOutside(dims[0] - 1, y, z);
            }
        const int n6[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        while (!q.empty()) {
            auto cur = q.front();
            q.pop_front();
            for (const auto &n : n6)
                pushOutside(cur[0] + n[0], cur[1] + n[1], cur[2] + n[2]);
        }
        std::vector<unsigned char> solid(voxelCount, 0);
        std::size_t solidCount = 0;
        for (std::size_t i = 0; i < voxelCount; ++i) {
            solid[i] = outside[i] ? 0 : 1;
            if (solid[i])
                ++solidCount;
        }
        if (solidCount < 10) {
            finish(sampledCloud, nullptr, nullptr, false, QStringLiteral("体素填充后实体过少，请调大术区或调整体素尺寸"));
            return;
        }

        reportProgress(45);
        reportLog(QStringLiteral("[3/8] 计算欧氏近似距离场"));
        constexpr double kInf = 1e18;
        std::vector<double> dist(voxelCount, kInf);
        using QueueItem = std::pair<double, std::size_t>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
        for (std::size_t i = 0; i < voxelCount; ++i) {
            if (!solid[i]) {
                dist[i] = 0.0;
                pq.push({0.0, i});
            }
        }
        std::vector<std::array<int, 3>> neighbors;
        std::vector<double> weights;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    neighbors.push_back({dx, dy, dz});
                    weights.push_back(std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)));
                }
        std::size_t processed = 0;
        while (!pq.empty()) {
            auto [d, idx] = pq.top();
            pq.pop();
            if (d != dist[idx])
                continue;
            const int x = static_cast<int>(idx % dims[0]);
            const int y = static_cast<int>((idx / dims[0]) % dims[1]);
            const int z = static_cast<int>(idx / (dims[0] * dims[1]));
            for (std::size_t ni = 0; ni < neighbors.size(); ++ni) {
                const int nx = x + neighbors[ni][0];
                const int ny = y + neighbors[ni][1];
                const int nz = z + neighbors[ni][2];
                if (!validVoxel(nx, ny, nz))
                    continue;
                const auto nidx = indexOf(nx, ny, nz);
                const double nd = d + weights[ni];
                if (nd < dist[nidx]) {
                    dist[nidx] = nd;
                    pq.push({nd, nidx});
                }
            }
            if (++processed % std::max<std::size_t>(1, voxelCount / 10) == 0)
                reportProgress(45 + static_cast<int>(18.0 * processed / voxelCount));
        }

        reportProgress(65);
        reportLog(QStringLiteral("[4/8] 提取中心候选点"));
        std::vector<double> solidDistances;
        solidDistances.reserve(solidCount);
        for (std::size_t i = 0; i < voxelCount; ++i)
            if (solid[i] && dist[i] > 0.0 && dist[i] < kInf)
                solidDistances.push_back(dist[i]);
        if (solidDistances.empty()) {
            finish(sampledCloud, nullptr, nullptr, false, QStringLiteral("距离场为空，无法提取中心候选点"));
            return;
        }
        std::sort(solidDistances.begin(), solidDistances.end());
        const std::size_t percentileIndex = std::min<std::size_t>(
            solidDistances.size() - 1,
            static_cast<std::size_t>((m_medialPercentile / 100.0) * (solidDistances.size() - 1)));
        const double threshold = solidDistances[percentileIndex];
        std::vector<AxisVec3> medialPoints;
        for (int z = 0; z < dims[2]; ++z)
            for (int y = 0; y < dims[1]; ++y)
                for (int x = 0; x < dims[0]; ++x) {
                    const auto idx = indexOf(x, y, z);
                    if (solid[idx] && dist[idx] >= threshold)
                        medialPoints.push_back(voxelToWorld(x, y, z));
                }
        if (medialPoints.size() < 10) {
            finish(sampledCloud, nullptr, nullptr, false, QStringLiteral("中心候选点过少，可降低中心候选百分位"));
            return;
        }
        reportLog(QStringLiteral("中心候选点数量=%1，阈值=%2").arg(medialPoints.size()).arg(threshold));

        reportProgress(72);
        reportLog(QStringLiteral("[5/8] 提取开口边界并拟合深度方向"));
        std::unordered_map<vtkIdType, std::vector<vtkIdType>> boundaryAdj;
        for (const auto &entry : edgeCount) {
            if (entry.second == 1) {
                boundaryAdj[entry.first.a].push_back(entry.first.b);
                boundaryAdj[entry.first.b].push_back(entry.first.a);
            }
        }
        std::vector<AxisVec3> openingPoints;
        int boundaryComponentCount = 0;
        if (!boundaryAdj.empty()) {
            std::unordered_map<vtkIdType, unsigned char> visited;
            std::vector<vtkIdType> bestComponent;
            for (const auto &entry : boundaryAdj) {
                const vtkIdType start = entry.first;
                if (visited[start])
                    continue;
                std::vector<vtkIdType> component;
                std::deque<vtkIdType> bfs;
                visited[start] = 1;
                bfs.push_back(start);
                while (!bfs.empty()) {
                    vtkIdType cur = bfs.front();
                    bfs.pop_front();
                    component.push_back(cur);
                    for (vtkIdType nxt : boundaryAdj[cur]) {
                        if (!visited[nxt]) {
                            visited[nxt] = 1;
                            bfs.push_back(nxt);
                        }
                    }
                }
                ++boundaryComponentCount;
                if (component.size() > bestComponent.size())
                    bestComponent = component;
            }
            openingPoints.reserve(bestComponent.size());
            for (vtkIdType id : bestComponent)
                if (id >= 0 && id < static_cast<vtkIdType>(vertices.size()))
                    openingPoints.push_back(vertices[static_cast<std::size_t>(id)]);
        }
        if (openingPoints.size() < 3) {
            reportLog(QStringLiteral("未检测到可靠开口边界，退化为 mesh 主轴方向"));
            openingPoints = sampledSurface;
        }
        AxisVec3 openingCenter = meanVec(openingPoints);
        AxisVec3 depthAxis = pcaDirection(openingPoints, false);
        if (normVec(depthAxis) <= 1e-12)
            depthAxis = pcaDirection(sampledSurface, true);
        AxisVec3 vecToInside = subVec(meshCenter, openingCenter);
        if (dotVec(depthAxis, vecToInside) < 0.0)
            depthAxis = mulVec(depthAxis, -1.0);
        depthAxis = normalizedVec(depthAxis);
        if (normVec(depthAxis) <= 1e-12) {
            finish(sampledCloud, nullptr, nullptr, false, QStringLiteral("深度方向计算失败"));
            return;
        }
        reportLog(QStringLiteral("入口几何诊断：边界组件=%1，选中边界点=%2，边界中心=[%3,%4,%5]，深度方向=[%6,%7,%8]")
            .arg(boundaryComponentCount).arg(openingPoints.size())
            .arg(openingCenter.x, 0, 'f', 6).arg(openingCenter.y, 0, 'f', 6)
            .arg(openingCenter.z, 0, 'f', 6).arg(depthAxis.x, 0, 'f', 6)
            .arg(depthAxis.y, 0, 'f', 6).arg(depthAxis.z, 0, 'f', 6));
        reportProgress(78);
        reportLog(QStringLiteral("[6/8] 剔除开口附近中心候选点"));
        std::vector<AxisVec3> filteredMedial;
        filteredMedial.reserve(medialPoints.size());
        for (const auto &p : medialPoints) {
            const double t = dotVec(subVec(p, openingCenter), depthAxis);
            if (m_excludeOpeningDistance <= 0.0 || t >= m_excludeOpeningDistance)
                filteredMedial.push_back(p);
        }
        if (filteredMedial.size() < 10)
            filteredMedial = medialPoints;

        reportProgress(83);
        reportLog(QStringLiteral("[7/8] 沿深度方向提取弯曲中心线"));
        std::vector<double> tValues;
        tValues.reserve(filteredMedial.size());
        double tMin = std::numeric_limits<double>::max();
        double tMax = std::numeric_limits<double>::lowest();
        for (const auto &p : filteredMedial) {
            const double t = dotVec(subVec(p, openingCenter), depthAxis);
            tValues.push_back(t);
            tMin = std::min(tMin, t);
            tMax = std::max(tMax, t);
        }
        const double tRange = tMax - tMin;
        double sectionHalfWidth = m_sectionHalfWidth;
        if (sectionHalfWidth <= 0.0)
            sectionHalfWidth = tRange / (2.5 * m_sectionCount);
        std::vector<AxisVec3> coarseCenterline;
        for (int s = m_excludeOpeningLayers; s < m_sectionCount; ++s) {
            const double tc = tMin + (tRange * s) / std::max(1, m_sectionCount - 1);
            AxisVec3 center;
            int count = 0;
            for (std::size_t i = 0; i < filteredMedial.size(); ++i) {
                if (std::abs(tValues[i] - tc) > sectionHalfWidth)
                    continue;
                center = addVec(center, filteredMedial[i]);
                ++count;
            }
            if (count >= 5)
                coarseCenterline.push_back(mulVec(center, 1.0 / count));
        }
        if (coarseCenterline.size() < 4) {
            finish(sampledCloud, nullptr, nullptr, false,
                   QStringLiteral("提取到的截面中心点过少，可调整中心候选百分位/分层数量/截面半宽"));
            return;
        }

        std::vector<AxisVec3> smoothed = coarseCenterline;
        if (m_smoothWindow > 1) {
            std::vector<AxisVec3> tmp;
            tmp.reserve(smoothed.size());
            const int halfWindow = m_smoothWindow / 2;
            for (int i = 0; i < static_cast<int>(smoothed.size()); ++i) {
                AxisVec3 center;
                int count = 0;
                for (int j = std::max(0, i - halfWindow); j <= std::min(static_cast<int>(smoothed.size()) - 1, i + halfWindow); ++j) {
                    center = addVec(center, smoothed[static_cast<std::size_t>(j)]);
                    ++count;
                }
                tmp.push_back(mulVec(center, 1.0 / count));
            }
            smoothed.swap(tmp);
        }
        std::vector<double> arc(smoothed.size(), 0.0);
        for (std::size_t i = 1; i < smoothed.size(); ++i)
            arc[i] = arc[i - 1] + normVec(subVec(smoothed[i], smoothed[i - 1]));
        std::vector<AxisVec3> curvedCenterline;
        if (arc.back() <= 1e-12) {
            curvedCenterline = smoothed;
        } else {
            curvedCenterline.reserve(static_cast<std::size_t>(m_smoothPointsNum));
            for (int i = 0; i < m_smoothPointsNum; ++i) {
                const double target = arc.back() * i / std::max(1, m_smoothPointsNum - 1);
                auto it = std::lower_bound(arc.begin(), arc.end(), target);
                std::size_t idx = static_cast<std::size_t>(std::distance(arc.begin(), it));
                if (idx == 0) {
                    curvedCenterline.push_back(smoothed.front());
                } else if (idx >= smoothed.size()) {
                    curvedCenterline.push_back(smoothed.back());
                } else {
                    const double denom = std::max(1e-12, arc[idx] - arc[idx - 1]);
                    const double u = (target - arc[idx - 1]) / denom;
                    curvedCenterline.push_back(addVec(smoothed[idx - 1], mulVec(subVec(smoothed[idx], smoothed[idx - 1]), u)));
                }
            }
        }

        reportProgress(94);
        reportLog(QStringLiteral("[8/8] 对弯曲中心线进行直轴拟合"));
        AxisVec3 lineCenter = meanVec(curvedCenterline);
        AxisVec3 lineDir = pcaDirection(curvedCenterline, true);
        if (normVec(lineDir) <= 1e-12) {
            finish(sampledCloud, pointsToPolyline(curvedCenterline), nullptr, false, QStringLiteral("直参考轴拟合失败"));
            return;
        }
        if (dotVec(lineDir, depthAxis) < 0.0)
            lineDir = mulVec(lineDir, -1.0);

        double lineMin = std::numeric_limits<double>::max();
        double lineMax = std::numeric_limits<double>::lowest();
        for (const auto &p : vertices) {
            const double t = dotVec(subVec(p, lineCenter), lineDir);
            lineMin = std::min(lineMin, t);
            lineMax = std::max(lineMax, t);
        }
        lineMin += m_outletRearExtent + m_safeMarginTop;
        lineMax -= m_safetyForwardExtent + m_safeMarginBottom;
        if (lineMax <= lineMin) {
            finish(sampledCloud, pointsToPolyline(curvedCenterline), nullptr, false,
                   QStringLiteral("末端前后占用与两端安全距离超过术区有效深度，请减小安全距离或更换末端规格"));
            return;
        }
        std::vector<AxisVec3> straightPoints = {
            addVec(lineCenter, mulVec(lineDir, lineMin)),
            addVec(lineCenter, mulVec(lineDir, lineMax))
        };

        vtkSmartPointer<vtkPolyData> straightAxis = pointsToPolyline(straightPoints);
        auto addVectorDiagnostic = [&straightAxis](const char *name, const AxisVec3 &value) {
            vtkNew<vtkDoubleArray> array;
            array->SetName(name);
            array->SetNumberOfComponents(3);
            const double tuple[3] = {value.x, value.y, value.z};
            array->InsertNextTuple(tuple);
            straightAxis->GetFieldData()->AddArray(array);
        };
        addVectorDiagnostic("OpeningBoundaryCenterXYZ", openingCenter);
        addVectorDiagnostic("OpeningBoundaryNormalXYZ", depthAxis);
        addVectorDiagnostic("OpeningDepthAxisXYZ", lineDir);
        addVectorDiagnostic("StraightAxisStartXYZ", straightPoints.front());
        addVectorDiagnostic("StraightAxisEndXYZ", straightPoints.back());
        vtkNew<vtkIntArray> boundaryComponents;
        boundaryComponents->SetName("OpeningBoundaryComponentCount");
        boundaryComponents->InsertNextValue(boundaryComponentCount);
        straightAxis->GetFieldData()->AddArray(boundaryComponents);
        vtkNew<vtkIntArray> selectedBoundaryPoints;
        selectedBoundaryPoints->SetName("OpeningBoundaryPointCount");
        selectedBoundaryPoints->InsertNextValue(static_cast<int>(openingPoints.size()));
        straightAxis->GetFieldData()->AddArray(selectedBoundaryPoints);

        reportProgress(100);
        reportLog(QStringLiteral("路径规划参考轴生成完成，弯曲中心线点=%1，安全轴长=%2m，直轴方向=[%3,%4,%5]")
            .arg(curvedCenterline.size()).arg(lineMax - lineMin, 0, 'f', 5)
            .arg(lineDir.x, 0, 'f', 4).arg(lineDir.y, 0, 'f', 4).arg(lineDir.z, 0, 'f', 4));
        reportLog(QStringLiteral("入口几何诊断：术区开口中心=[%1,%2,%3]，中心线拟合中心=[%4,%5,%6]")
            .arg(openingCenter.x, 0, 'f', 6).arg(openingCenter.y, 0, 'f', 6)
            .arg(openingCenter.z, 0, 'f', 6)
            .arg(lineCenter.x, 0, 'f', 6).arg(lineCenter.y, 0, 'f', 6)
            .arg(lineCenter.z, 0, 'f', 6));
        reportLog(QStringLiteral("入口几何诊断：蓝轴起点=[%1,%2,%3]，终点=[%4,%5,%6]")
            .arg(straightPoints.front().x, 0, 'f', 6)
            .arg(straightPoints.front().y, 0, 'f', 6)
            .arg(straightPoints.front().z, 0, 'f', 6)
            .arg(straightPoints.back().x, 0, 'f', 6)
            .arg(straightPoints.back().y, 0, 'f', 6)
            .arg(straightPoints.back().z, 0, 'f', 6));
        finish(sampledCloud, pointsToPolyline(curvedCenterline), straightAxis,
               true, QStringLiteral("路径规划参考轴生成完成"));
    } catch (const std::exception &e) {
        finish(nullptr, nullptr, nullptr, false, QStringLiteral("路径规划异常: %1").arg(e.what()));
    } catch (...) {
        finish(nullptr, nullptr, nullptr, false, QStringLiteral("路径规划发生未知异常"));
    }
}

// ==========================================
// SlicePlanningTask 实现
// ==========================================

SlicePlanningTask::SlicePlanningTask(const QString &sourceName,
                                     vtkSmartPointer<vtkPolyData> mesh,
                                     vtkSmartPointer<vtkPolyData> straightAxis,
                                     vtkSmartPointer<vtkPolyData> curvedAxis,
                                     double sliceSpacing,
                                     double outletRearExtent,
                                     double safetyForwardExtent,
                                     double safeMarginBottom,
                                     double safeMarginTop,
                                     double planeScaleRatio,
                                     QObject *receiver)
    : m_sourceName(sourceName),
      m_mesh(mesh),
      m_straightAxis(straightAxis),
      m_curvedAxis(curvedAxis),
      m_sliceSpacing(sliceSpacing),
      m_outletRearExtent(std::max(0.0, outletRearExtent)),
      m_safetyForwardExtent(std::max(0.0, safetyForwardExtent)),
      m_safeMarginBottom(safeMarginBottom),
      m_safeMarginTop(safeMarginTop),
      m_planeScaleRatio(planeScaleRatio),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void SlicePlanningTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onSlicePlanningProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(int, progress));
}

void SlicePlanningTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onSlicePlanningLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(QString, message));
}

void SlicePlanningTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> planes,
                         vtkSmartPointer<vtkPolyData> firstPlane,
                         vtkSmartPointer<vtkPolyData> bbox,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onSlicePlanningFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, planes),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, firstPlane),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, bbox),
                                  Q_ARG(bool, ok), Q_ARG(QString, message));
    };

    try {
        if (!m_mesh || m_mesh->GetNumberOfPoints() < 3) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("缺少重建 mesh"));
            return;
        }
        if (!m_straightAxis || m_straightAxis->GetNumberOfPoints() < 2) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("缺少直参考轴"));
            return;
        }
        if (m_sliceSpacing <= 0.0 || m_safeMarginBottom < 0.0 ||
            m_safeMarginTop < 0.0 || m_planeScaleRatio <= 0.0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("切片参数无效"));
            return;
        }

        reportProgress(5);
        reportLog(QStringLiteral("开始生成切片平面：间距=%1，喷口后向占用=%2，安全点前向占用=%3，底部额外余量=%4，开口额外余量=%5，平面缩放=%6")
            .arg(m_sliceSpacing).arg(m_outletRearExtent).arg(m_safetyForwardExtent)
            .arg(m_safeMarginBottom).arg(m_safeMarginTop).arg(m_planeScaleRatio));

        double p0raw[3];
        double p1raw[3];
        m_straightAxis->GetPoint(0, p0raw);
        m_straightAxis->GetPoint(m_straightAxis->GetNumberOfPoints() - 1, p1raw);
        AxisVec3 p0 = makeVec(p0raw[0], p0raw[1], p0raw[2]);
        AxisVec3 p1 = makeVec(p1raw[0], p1raw[1], p1raw[2]);
        AxisVec3 a = normalizedVec(subVec(p1, p0));
        if (normVec(a) <= 1e-12) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("直参考轴长度为 0"));
            return;
        }

        if (m_curvedAxis && m_curvedAxis->GetNumberOfPoints() >= 2) {
            double first[3];
            double last[3];
            m_curvedAxis->GetPoint(0, first);
            m_curvedAxis->GetPoint(m_curvedAxis->GetNumberOfPoints() - 1, last);
            AxisVec3 depthHint = subVec(makeVec(last[0], last[1], last[2]),
                                        makeVec(first[0], first[1], first[2]));
            if (dotVec(a, depthHint) < 0.0)
                a = mulVec(a, -1.0);
        }

        const AxisVec3 o = mulVec(addVec(p0, p1), 0.5);
        std::vector<AxisVec3> meshPoints;
        std::vector<AxisVec3> projectedPoints;
        meshPoints.reserve(static_cast<std::size_t>(m_mesh->GetNumberOfPoints()));
        projectedPoints.reserve(static_cast<std::size_t>(m_mesh->GetNumberOfPoints()));
        for (vtkIdType i = 0; i < m_mesh->GetNumberOfPoints(); ++i) {
            double raw[3];
            m_mesh->GetPoint(i, raw);
            AxisVec3 p = makeVec(raw[0], raw[1], raw[2]);
            meshPoints.push_back(p);
            AxisVec3 rel = subVec(p, o);
            projectedPoints.push_back(subVec(rel, mulVec(a, dotVec(rel, a))));
        }

        reportProgress(20);
        AxisVec3 u = pcaDirection(projectedPoints, true);
        u = normalizedVec(subVec(u, mulVec(a, dotVec(u, a))));
        if (normVec(u) <= 1e-12) {
            const AxisVec3 reference = std::abs(a.z) < 0.9 ? makeVec(0.0, 0.0, 1.0)
                                                           : makeVec(0.0, 1.0, 0.0);
            u = normalizedVec(crossVec(reference, a));
        }
        AxisVec3 v = normalizedVec(crossVec(a, u));
        u = normalizedVec(crossVec(v, a));

        double uMin = std::numeric_limits<double>::max();
        double uMax = std::numeric_limits<double>::lowest();
        double vMin = std::numeric_limits<double>::max();
        double vMax = std::numeric_limits<double>::lowest();
        double aMin = std::numeric_limits<double>::max();
        double aMax = std::numeric_limits<double>::lowest();
        for (const auto &p : meshPoints) {
            AxisVec3 rel = subVec(p, o);
            const double lu = dotVec(rel, u);
            const double lv = dotVec(rel, v);
            const double la = dotVec(rel, a);
            uMin = std::min(uMin, lu); uMax = std::max(uMax, lu);
            vMin = std::min(vMin, lv); vMax = std::max(vMax, lv);
            aMin = std::min(aMin, la); aMax = std::max(aMax, la);
        }

        const double openingLimit = aMin + m_safeMarginTop + m_outletRearExtent;
        const double bottomLimit = aMax - m_safeMarginBottom - m_safetyForwardExtent;
        if (bottomLimit <= openingLimit) {
            finish(nullptr, nullptr, nullptr, false,
                   QStringLiteral("有效切片区间无效，请减小安全距离"));
            return;
        }

        const double firstSlice = bottomLimit;
        if (firstSlice < openingLimit) {
            finish(nullptr, nullptr, nullptr, false,
                   QStringLiteral("首个切片平面超出有效范围，请减小切片间距或安全距离"));
            return;
        }

        std::vector<double> positions;
        for (double pos = firstSlice; pos >= openingLimit - 1e-12; pos -= m_sliceSpacing)
            positions.push_back(pos);
        if (positions.empty()) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("未生成任何切片平面"));
            return;
        }

        const double uCenter = 0.5 * (uMin + uMax);
        const double vCenter = 0.5 * (vMin + vMax);
        const double uHalf = 0.5 * (uMax - uMin) * m_planeScaleRatio;
        const double vHalf = 0.5 * (vMax - vMin) * m_planeScaleRatio;
        auto localToWorld = [&](double lu, double lv, double la) {
            return addVec(o, addVec(mulVec(u, lu), addVec(mulVec(v, lv), mulVec(a, la))));
        };

        auto buildPlanes = [&](std::size_t begin, std::size_t end) {
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
            for (std::size_t k = begin; k < end; ++k) {
                const double pos = positions[k];
                const AxisVec3 corners[4] = {
                    localToWorld(uCenter - uHalf, vCenter - vHalf, pos),
                    localToWorld(uCenter + uHalf, vCenter - vHalf, pos),
                    localToWorld(uCenter + uHalf, vCenter + vHalf, pos),
                    localToWorld(uCenter - uHalf, vCenter + vHalf, pos)
                };
                vtkIdType ids[4];
                for (int i = 0; i < 4; ++i)
                    ids[i] = points->InsertNextPoint(corners[i].x, corners[i].y, corners[i].z);
                polys->InsertNextCell(4, ids);
            }
            vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
            result->SetPoints(points);
            result->SetPolys(polys);
            return result;
        };

        reportProgress(65);
        vtkSmartPointer<vtkPolyData> firstPlane = buildPlanes(0, 1);
        vtkSmartPointer<vtkPolyData> otherPlanes = buildPlanes(1, positions.size());

        const AxisVec3 bboxCorners[8] = {
            localToWorld(uMin, vMin, aMin), localToWorld(uMax, vMin, aMin),
            localToWorld(uMax, vMax, aMin), localToWorld(uMin, vMax, aMin),
            localToWorld(uMin, vMin, aMax), localToWorld(uMax, vMin, aMax),
            localToWorld(uMax, vMax, aMax), localToWorld(uMin, vMax, aMax)
        };
        const int edges[12][2] = {
            {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
        };
        vtkSmartPointer<vtkPoints> bboxPoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> bboxLines = vtkSmartPointer<vtkCellArray>::New();
        for (const auto &corner : bboxCorners)
            bboxPoints->InsertNextPoint(corner.x, corner.y, corner.z);
        for (const auto &edge : edges) {
            vtkIdType ids[2] = {edge[0], edge[1]};
            bboxLines->InsertNextCell(2, ids);
        }
        vtkSmartPointer<vtkPolyData> bbox = vtkSmartPointer<vtkPolyData>::New();
        bbox->SetPoints(bboxPoints);
        bbox->SetLines(bboxLines);

        reportProgress(100);
        reportLog(QStringLiteral("切片平面生成完成：数量=%1，开口端=%2，底部端=%3，首切片=%4")
            .arg(positions.size()).arg(openingLimit).arg(bottomLimit).arg(firstSlice));
        finish(otherPlanes, firstPlane, bbox, true,
               QStringLiteral("切片平面生成完成，共 %1 个").arg(positions.size()));
    } catch (const std::exception &e) {
        finish(nullptr, nullptr, nullptr, false, QStringLiteral("切片计算异常: %1").arg(e.what()));
    } catch (...) {
        finish(nullptr, nullptr, nullptr, false, QStringLiteral("切片计算发生未知异常"));
    }
}

// ==========================================
// SliceContourTask 实现
// ==========================================

SliceContourTask::SliceContourTask(const QString &sourceName,
                                   vtkSmartPointer<vtkPolyData> surgicalMesh,
                                   vtkSmartPointer<vtkPolyData> slicePlanes,
                                   vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                   QObject *receiver)
    : m_sourceName(sourceName),
      m_surgicalMesh(surgicalMesh),
      m_slicePlanes(slicePlanes),
      m_firstSlicePlane(firstSlicePlane),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void SliceContourTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onSliceContourProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(int, progress));
}

void SliceContourTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onSliceContourLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(QString, message));
}

void SliceContourTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> contours,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onSliceContourFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, contours),
                                  Q_ARG(bool, ok), Q_ARG(QString, message));
    };

    try {
        if (!m_surgicalMesh || m_surgicalMesh->GetNumberOfCells() == 0) {
            finish(nullptr, false, QStringLiteral("缺少术区 mesh"));
            return;
        }

        const vtkIdType firstCount = m_firstSlicePlane ? m_firstSlicePlane->GetNumberOfCells() : 0;
        const vtkIdType otherCount = m_slicePlanes ? m_slicePlanes->GetNumberOfCells() : 0;
        const vtkIdType totalCount = firstCount + otherCount;
        if (totalCount == 0) {
            finish(nullptr, false, QStringLiteral("请先生成切片平面"));
            return;
        }

        reportProgress(2);
        reportLog(QStringLiteral("开始计算 %1 个切片平面与术区 mesh 的交线").arg(totalCount));

        vtkNew<vtkAppendPolyData> append;
        vtkIdType processed = 0;
        vtkIdType nonEmptyContours = 0;
        int sliceIndex = 0;

        auto cutPlanes = [&](vtkPolyData *planeData) {
            if (!planeData)
                return;
            for (vtkIdType cellId = 0; cellId < planeData->GetNumberOfCells(); ++cellId) {
                vtkCell *cell = planeData->GetCell(cellId);
                if (!cell || cell->GetNumberOfPoints() < 3) {
                    ++processed;
                    ++sliceIndex;
                    continue;
                }

                double p0raw[3];
                double p1raw[3];
                double p2raw[3];
                planeData->GetPoint(cell->GetPointId(0), p0raw);
                planeData->GetPoint(cell->GetPointId(1), p1raw);
                planeData->GetPoint(cell->GetPointId(2), p2raw);
                const AxisVec3 p0 = makeVec(p0raw[0], p0raw[1], p0raw[2]);
                const AxisVec3 e1 = subVec(makeVec(p1raw[0], p1raw[1], p1raw[2]), p0);
                const AxisVec3 e2 = subVec(makeVec(p2raw[0], p2raw[1], p2raw[2]), p0);
                const AxisVec3 normal = normalizedVec(crossVec(e1, e2));
                if (normVec(normal) <= 1e-12) {
                    ++processed;
                    ++sliceIndex;
                    continue;
                }

                vtkNew<vtkPlane> plane;
                plane->SetOrigin(p0raw);
                plane->SetNormal(normal.x, normal.y, normal.z);

                vtkNew<vtkCutter> cutter;
                cutter->SetInputData(m_surgicalMesh);
                cutter->SetCutFunction(plane);
                cutter->GenerateTrianglesOff();

                vtkNew<vtkStripper> stripper;
                stripper->SetInputConnection(cutter->GetOutputPort());
                stripper->JoinContiguousSegmentsOn();
                stripper->Update();

                vtkPolyData *output = stripper->GetOutput();
                if (output && output->GetNumberOfPoints() >= 2 && output->GetNumberOfLines() > 0) {
                    vtkSmartPointer<vtkPolyData> contour = vtkSmartPointer<vtkPolyData>::New();
                    contour->DeepCopy(output);
                    vtkNew<vtkIntArray> sliceIds;
                    sliceIds->SetName("SliceIndex");
                    sliceIds->SetNumberOfTuples(contour->GetNumberOfCells());
                    sliceIds->FillComponent(0, sliceIndex);
                    contour->GetCellData()->AddArray(sliceIds);
                    append->AddInputData(contour);
                    ++nonEmptyContours;
                }

                ++processed;
                ++sliceIndex;
                reportProgress(5 + static_cast<int>(90.0 * processed / totalCount));
            }
        };

        cutPlanes(m_firstSlicePlane);
        cutPlanes(m_slicePlanes);
        if (nonEmptyContours == 0) {
            finish(nullptr, false, QStringLiteral("所有切片平面均未与术区 mesh 相交"));
            return;
        }

        append->Update();
        vtkNew<vtkCleanPolyData> clean;
        clean->SetInputConnection(append->GetOutputPort());
        clean->PointMergingOn();
        clean->Update();

        vtkSmartPointer<vtkPolyData> contours = vtkSmartPointer<vtkPolyData>::New();
        contours->DeepCopy(clean->GetOutput());
        reportProgress(100);
        reportLog(QStringLiteral("切片轮廓生成完成：有效层=%1，轮廓点=%2，线单元=%3")
            .arg(nonEmptyContours).arg(contours->GetNumberOfPoints()).arg(contours->GetNumberOfLines()));
        finish(contours, true, QStringLiteral("切片轮廓生成完成"));
    } catch (const std::exception &e) {
        finish(nullptr, false, QStringLiteral("切片轮廓计算异常: %1").arg(e.what()));
    } catch (...) {
        finish(nullptr, false, QStringLiteral("切片轮廓计算发生未知异常"));
    }
}

// ==========================================
// ContourFittingTask 实现
// ==========================================

ContourFittingTask::ContourFittingTask(const QString &sourceName,
                                       vtkSmartPointer<vtkPolyData> contours,
                                       vtkSmartPointer<vtkPolyData> straightAxis,
                                       vtkSmartPointer<vtkPolyData> curvedAxis,
                                       int fitPointCount,
                                       double coverageThreshold,
                                       int smoothWindow,
                                       double angleBinDeg,
                                       bool trimOpenEnds,
                                       int endCheckCount,
                                       double curvaturePeakRatio,
                                       double curvatureRecoverRatio,
                                       int maxTrimCount,
                                       int minKeepPointCount,
                                       QObject *receiver)
    : m_sourceName(sourceName),
      m_contours(contours),
      m_straightAxis(straightAxis),
      m_curvedAxis(curvedAxis),
      m_fitPointCount(std::clamp(fitPointCount, 40, 1200)),
      m_coverageThreshold(std::clamp(coverageThreshold, 0.1, 1.0)),
      m_smoothWindow(std::clamp(smoothWindow | 1, 1, 51)),
      m_angleBinDeg(std::clamp(angleBinDeg, 0.1, 30.0)),
      m_trimOpenEnds(trimOpenEnds),
      m_endCheckCount(std::clamp(endCheckCount, 1, 5000)),
      m_curvaturePeakRatio(std::clamp(curvaturePeakRatio, 0.01, 100.0)),
      m_curvatureRecoverRatio(std::clamp(curvatureRecoverRatio, 0.0001, 10.0)),
      m_maxTrimCount(std::clamp(maxTrimCount, 0, 2000)),
      m_minKeepPointCount(std::clamp(minKeepPointCount, 4, 1200)),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void ContourFittingTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onContourFittingProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(int, progress));
}

void ContourFittingTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onContourFittingLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(QString, message));
}

void ContourFittingTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> fitted,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onContourFittingFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, fitted),
                                  Q_ARG(bool, ok), Q_ARG(QString, message));
    };

    try {
        if (!m_contours || m_contours->GetNumberOfLines() == 0) {
            finish(nullptr, false, QStringLiteral("请先生成切片轮廓"));
            return;
        }
        if (!m_straightAxis || m_straightAxis->GetNumberOfPoints() < 2) {
            finish(nullptr, false, QStringLiteral("缺少直参考轴"));
            return;
        }

        reportProgress(2);
        reportLog(QStringLiteral("开始 B 样条拟合：输出点=%1，闭合阈值=%2，平滑窗口=%3，角度分箱=%4度，端部清理=%5")
            .arg(m_fitPointCount).arg(m_coverageThreshold).arg(m_smoothWindow)
            .arg(m_angleBinDeg).arg(m_trimOpenEnds));
        if (m_trimOpenEnds) {
            reportLog(QStringLiteral("端部曲率参数：检查点=%1，峰值比=%2，恢复比=%3，最大裁剪=%4，最少保留=%5")
                .arg(m_endCheckCount).arg(m_curvaturePeakRatio)
                .arg(m_curvatureRecoverRatio).arg(m_maxTrimCount)
                .arg(m_minKeepPointCount));
        }

        vtkIntArray *sliceIds = vtkIntArray::SafeDownCast(
            m_contours->GetCellData()->GetArray("SliceIndex"));
        std::map<int, std::vector<AxisVec3>> layers;
        for (vtkIdType cellId = 0; cellId < m_contours->GetNumberOfCells(); ++cellId) {
            vtkCell *cell = m_contours->GetCell(cellId);
            if (!cell || cell->GetNumberOfPoints() < 4)
                continue;
            const int sliceId = sliceIds && sliceIds->GetNumberOfTuples() > cellId
                ? sliceIds->GetValue(cellId)
                : static_cast<int>(cellId);
            std::vector<AxisVec3> points;
            points.reserve(static_cast<std::size_t>(cell->GetNumberOfPoints()));
            for (vtkIdType i = 0; i < cell->GetNumberOfPoints(); ++i) {
                double p[3];
                m_contours->GetPoint(cell->GetPointId(i), p);
                points.push_back(makeVec(p[0], p[1], p[2]));
            }
            if (points.size() > layers[sliceId].size())
                layers[sliceId] = std::move(points);
        }
        if (layers.empty()) {
            finish(nullptr, false, QStringLiteral("切片轮廓中没有可拟合的连续线"));
            return;
        }

        double p0raw[3];
        double p1raw[3];
        m_straightAxis->GetPoint(0, p0raw);
        m_straightAxis->GetPoint(m_straightAxis->GetNumberOfPoints() - 1, p1raw);
        const AxisVec3 p0 = makeVec(p0raw[0], p0raw[1], p0raw[2]);
        const AxisVec3 p1 = makeVec(p1raw[0], p1raw[1], p1raw[2]);
        AxisVec3 a = normalizedVec(subVec(p1, p0));
        if (m_curvedAxis && m_curvedAxis->GetNumberOfPoints() >= 2) {
            double first[3];
            double last[3];
            m_curvedAxis->GetPoint(0, first);
            m_curvedAxis->GetPoint(m_curvedAxis->GetNumberOfPoints() - 1, last);
            if (dotVec(a, subVec(makeVec(last[0], last[1], last[2]),
                                 makeVec(first[0], first[1], first[2]))) < 0.0) {
                a = mulVec(a, -1.0);
            }
        }
        const AxisVec3 o = mulVec(addVec(p0, p1), 0.5);
        const AxisVec3 reference = std::abs(a.z) < 0.9 ? makeVec(0.0, 0.0, 1.0)
                                                       : makeVec(0.0, 1.0, 0.0);
        AxisVec3 u = normalizedVec(crossVec(reference, a));
        AxisVec3 v = normalizedVec(crossVec(a, u));
        auto worldToLocal = [&](const AxisVec3 &p) {
            const AxisVec3 rel = subVec(p, o);
            return makeVec(dotVec(rel, u), dotVec(rel, v), dotVec(rel, a));
        };
        auto localToWorld = [&](const AxisVec3 &p) {
            return addVec(o, addVec(mulVec(u, p.x), addVec(mulVec(v, p.y), mulVec(a, p.z))));
        };

        vtkSmartPointer<vtkPoints> fittedPoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> fittedLines = vtkSmartPointer<vtkCellArray>::New();
        vtkNew<vtkIntArray> fittedSliceIds;
        fittedSliceIds->SetName("SliceIndex");
        vtkNew<vtkIntArray> closedFlags;
        closedFlags->SetName("ClosedLayer");

        int fittedLayerCount = 0;
        int closedLayerCount = 0;
        int layerOrdinal = 0;
        for (const auto &entry : layers) {
            ++layerOrdinal;
            std::vector<AxisVec3> localPoints;
            localPoints.reserve(entry.second.size());
            for (const auto &p : entry.second)
                localPoints.push_back(worldToLocal(p));

            struct AngularPoint { double angle; AxisVec3 point; };
            std::vector<AngularPoint> angular;
            angular.reserve(localPoints.size());
            for (const auto &p : localPoints) {
                double angle = std::atan2(p.y, p.x) * 180.0 / vtkMath::Pi();
                if (angle < 0.0)
                    angle += 360.0;
                angular.push_back({angle, p});
            }
            std::sort(angular.begin(), angular.end(), [](const auto &lhs, const auto &rhs) {
                return lhs.angle < rhs.angle;
            });
            if (angular.size() < 4)
                continue;

            double maxGap = -1.0;
            std::size_t maxGapIndex = 0;
            for (std::size_t i = 0; i < angular.size(); ++i) {
                const std::size_t j = (i + 1) % angular.size();
                const double nextAngle = j == 0 ? angular[j].angle + 360.0 : angular[j].angle;
                const double gap = nextAngle - angular[i].angle;
                if (gap > maxGap) {
                    maxGap = gap;
                    maxGapIndex = i;
                }
            }
            const double coverage = std::clamp(1.0 - maxGap / 360.0, 0.0, 1.0);
            const bool closed = coverage >= m_coverageThreshold;

            if (!closed) {
                std::rotate(angular.begin(), angular.begin() + ((maxGapIndex + 1) % angular.size()), angular.end());
                for (std::size_t i = 1; i < angular.size(); ++i)
                    while (angular[i].angle < angular[i - 1].angle)
                        angular[i].angle += 360.0;
            }

            std::map<int, std::pair<AxisVec3, int>> bins;
            const double baseAngle = closed ? 0.0 : angular.front().angle;
            for (const auto &item : angular) {
                const int binId = static_cast<int>(std::floor((item.angle - baseAngle) / m_angleBinDeg));
                auto &bin = bins[binId];
                bin.first = addVec(bin.first, item.point);
                ++bin.second;
            }
            std::vector<AxisVec3> representatives;
            representatives.reserve(bins.size());
            for (const auto &bin : bins)
                if (bin.second.second > 0)
                    representatives.push_back(mulVec(bin.second.first, 1.0 / bin.second.second));
            if (representatives.size() < 4)
                continue;

            if (m_smoothWindow > 1 && representatives.size() > static_cast<std::size_t>(m_smoothWindow)) {
                std::vector<AxisVec3> smoothed;
                smoothed.reserve(representatives.size());
                const int half = m_smoothWindow / 2;
                for (int i = 0; i < static_cast<int>(representatives.size()); ++i) {
                    AxisVec3 sum;
                    int count = 0;
                    for (int offset = -half; offset <= half; ++offset) {
                        int idx = i + offset;
                        if (closed) {
                            idx %= static_cast<int>(representatives.size());
                            if (idx < 0)
                                idx += static_cast<int>(representatives.size());
                        } else if (idx < 0 || idx >= static_cast<int>(representatives.size())) {
                            continue;
                        }
                        sum = addVec(sum, representatives[static_cast<std::size_t>(idx)]);
                        ++count;
                    }
                    smoothed.push_back(mulVec(sum, 1.0 / std::max(1, count)));
                }
                representatives.swap(smoothed);
            }

            vtkNew<vtkPoints> controlPoints;
            for (const auto &p : representatives)
                controlPoints->InsertNextPoint(p.x, p.y, p.z);
            vtkNew<vtkParametricSpline> spline;
            spline->SetPoints(controlPoints);
            spline->SetClosed(closed ? 1 : 0);
            vtkNew<vtkParametricFunctionSource> splineSource;
            splineSource->SetParametricFunction(spline);
            splineSource->SetUResolution(m_fitPointCount - 1);
            splineSource->Update();

            std::vector<AxisVec3> fittedLocal;
            vtkPoints *splinePoints = splineSource->GetOutput()->GetPoints();
            if (!splinePoints || splinePoints->GetNumberOfPoints() < 4)
                continue;
            fittedLocal.reserve(static_cast<std::size_t>(splinePoints->GetNumberOfPoints()));
            for (vtkIdType i = 0; i < splinePoints->GetNumberOfPoints(); ++i) {
                double p[3];
                splinePoints->GetPoint(i, p);
                fittedLocal.push_back(makeVec(p[0], p[1], p[2]));
            }

            if (!closed && m_trimOpenEnds && fittedLocal.size() >= 10) {
                std::vector<double> curvature(fittedLocal.size(), 0.0);
                for (std::size_t i = 1; i + 1 < fittedLocal.size(); ++i) {
                    AxisVec3 d1 = subVec(fittedLocal[i], fittedLocal[i - 1]);
                    AxisVec3 d2 = subVec(fittedLocal[i + 1], fittedLocal[i]);
                    const double l1 = normVec(d1);
                    const double l2 = normVec(d2);
                    if (l1 > 1e-12 && l2 > 1e-12) {
                        const double c = std::clamp(dotVec(d1, d2) / (l1 * l2), -1.0, 1.0);
                        curvature[i] = std::acos(c) / std::max(1e-12, 0.5 * (l1 + l2));
                    }
                }
                curvature.front() = curvature[1];
                curvature.back() = curvature[curvature.size() - 2];
                const int pointCount = static_cast<int>(fittedLocal.size());
                const int endCheck = std::min(m_endCheckCount, pointCount / 3);
                const int maxTrim = std::min(m_maxTrimCount, pointCount / 3);
                std::vector<double> middle(curvature.begin() + endCheck, curvature.end() - endCheck);
                if (!middle.empty()) {
                    std::nth_element(middle.begin(), middle.begin() + middle.size() / 2, middle.end());
                    double baseCurvature = middle[middle.size() / 2];
                    if (baseCurvature < 1e-12) {
                        baseCurvature = std::accumulate(middle.begin(), middle.end(), 0.0) /
                                        static_cast<double>(middle.size()) + 1e-12;
                    }
                    const double peakThreshold = m_curvaturePeakRatio * baseCurvature;
                    const double recoverThreshold = m_curvatureRecoverRatio * baseCurvature;

                    int trimStart = 0;
                    auto headPeak = std::find_if(curvature.begin(), curvature.begin() + endCheck,
                        [peakThreshold](double value) { return value > peakThreshold; });
                    if (headPeak != curvature.begin() + endCheck) {
                        const int peakIndex = static_cast<int>(std::distance(curvature.begin(), headPeak));
                        for (int i = peakIndex; i < endCheck; ++i) {
                            if (curvature[static_cast<std::size_t>(i)] <= recoverThreshold) {
                                trimStart = i;
                                break;
                            }
                        }
                        if (trimStart == 0)
                            trimStart = std::min(endCheck, maxTrim);
                    }
                    trimStart = std::min(trimStart, maxTrim);

                    int trimEnd = 0;
                    int tailPeakIndex = -1;
                    for (int i = pointCount - 1; i >= pointCount - endCheck; --i) {
                        if (curvature[static_cast<std::size_t>(i)] > peakThreshold) {
                            tailPeakIndex = i;
                            break;
                        }
                    }
                    if (tailPeakIndex >= 0) {
                        for (int i = tailPeakIndex; i >= pointCount - endCheck; --i) {
                            if (curvature[static_cast<std::size_t>(i)] <= recoverThreshold) {
                                trimEnd = pointCount - 1 - i;
                                break;
                            }
                        }
                        if (trimEnd == 0)
                            trimEnd = std::min(endCheck, maxTrim);
                    }
                    trimEnd = std::min(trimEnd, maxTrim);

                    if (pointCount - trimStart - trimEnd >= m_minKeepPointCount) {
                        fittedLocal = std::vector<AxisVec3>(fittedLocal.begin() + trimStart,
                                                            fittedLocal.end() - trimEnd);
                    }
                }
            }

            std::vector<vtkIdType> ids;
            ids.reserve(fittedLocal.size() + (closed ? 1 : 0));
            for (const auto &p : fittedLocal) {
                const AxisVec3 world = localToWorld(p);
                ids.push_back(fittedPoints->InsertNextPoint(world.x, world.y, world.z));
            }
            if (closed && !ids.empty())
                ids.push_back(ids.front());
            if (ids.size() < 2)
                continue;
            fittedLines->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
            fittedSliceIds->InsertNextValue(entry.first);
            closedFlags->InsertNextValue(closed ? 1 : 0);
            ++fittedLayerCount;
            if (closed)
                ++closedLayerCount;

            reportProgress(5 + static_cast<int>(90.0 * layerOrdinal / layers.size()));
            reportLog(QStringLiteral("第 %1 层：%2，覆盖率=%3，候选点=%4，代表点=%5，输出点=%6")
                .arg(entry.first + 1)
                .arg(closed ? QStringLiteral("闭合") : QStringLiteral("开口"))
                .arg(coverage, 0, 'f', 3)
                .arg(entry.second.size()).arg(representatives.size()).arg(fittedLocal.size()));
        }

        if (fittedLayerCount == 0) {
            finish(nullptr, false, QStringLiteral("没有切片轮廓拟合成功"));
            return;
        }

        vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
        result->SetPoints(fittedPoints);
        result->SetLines(fittedLines);
        result->GetCellData()->AddArray(fittedSliceIds);
        result->GetCellData()->AddArray(closedFlags);
        reportProgress(100);
        reportLog(QStringLiteral("B 样条拟合完成：有效层=%1，闭合层=%2，开口层=%3")
            .arg(fittedLayerCount).arg(closedLayerCount).arg(fittedLayerCount - closedLayerCount));
        finish(result, true, QStringLiteral("B 样条轮廓拟合完成"));
    } catch (const std::exception &e) {
        finish(nullptr, false, QStringLiteral("B 样条拟合异常: %1").arg(e.what()));
    } catch (...) {
        finish(nullptr, false, QStringLiteral("B 样条拟合发生未知异常"));
    }
}

// ==========================================
// EqualDosePathTask 实现
// ==========================================

EqualDosePathTask::EqualDosePathTask(const QString &sourceName,
                                     vtkSmartPointer<vtkPolyData> fittedContours,
                                     vtkSmartPointer<vtkPolyData> straightAxis,
                                     vtkSmartPointer<vtkPolyData> curvedAxis,
                                     double nozzleVerticalLength,
                                     double sprayRodRadius,
                                     double safetyClearance,
                                     int closedSamples,
                                     int openSamples,
                                     double connectionRefAngleDeg,
                                     bool useTopOpenEndpoint,
                                     const QString &openEndpointMode,
                                     const QString &manualRegionRanges,
                                     double manualZeroOffsetDeg,
                                     const QString &angleViewDirection,
                                     const QString &angleIncreaseDirection,
                                     QObject *receiver)
    : m_sourceName(sourceName),
      m_fittedContours(fittedContours),
      m_straightAxis(straightAxis),
      m_curvedAxis(curvedAxis),
      m_nozzleVerticalLength(std::clamp(nozzleVerticalLength, 0.0001, 0.2)),
      m_sprayRodRadius(std::clamp(sprayRodRadius, 0.0, 0.1)),
      m_safetyClearance(std::clamp(safetyClearance, 0.0, 0.1)),
      m_closedSamples(std::clamp(closedSamples, 8, 4000)),
      m_openSamples(std::clamp(openSamples, 8, 4000)),
      m_connectionRefAngleDeg(wrapTo360(connectionRefAngleDeg)),
      m_useTopOpenEndpoint(useTopOpenEndpoint),
      m_openEndpointMode(openEndpointMode.trimmed().toLower()),
      m_manualRegionRanges(manualRegionRanges),
      m_manualZeroOffsetDeg(manualZeroOffsetDeg),
      m_angleViewDirection(angleViewDirection.trimmed().toLower()),
      m_angleIncreaseDirection(angleIncreaseDirection.trimmed().toLower()),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void EqualDosePathTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onEqualDosePathProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(int, progress));
}

void EqualDosePathTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onEqualDosePathLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(QString, message));
}

void EqualDosePathTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> surface,
                         vtkSmartPointer<vtkPolyData> path,
                         vtkSmartPointer<vtkPolyData> connections,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onEqualDosePathFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, surface),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, path),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, connections),
                                  Q_ARG(bool, ok), Q_ARG(QString, message));
    };

    try {
        if (!m_fittedContours || m_fittedContours->GetNumberOfLines() == 0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("请先拟合切片轮廓"));
            return;
        }
        if (!m_straightAxis || m_straightAxis->GetNumberOfPoints() < 2) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("缺少直参考轴"));
            return;
        }
        if (m_openEndpointMode != QStringLiteral("start") &&
            m_openEndpointMode != QStringLiteral("end")) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("开口层连接端只能是 start 或 end"));
            return;
        }
        if (m_angleViewDirection != QStringLiteral("+a") &&
            m_angleViewDirection != QStringLiteral("-a")) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("角度观察方向只能是 +a 或 -a"));
            return;
        }
        if (m_angleIncreaseDirection != QStringLiteral("ccw") &&
            m_angleIncreaseDirection != QStringLiteral("cw")) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("角度增长方向只能是 ccw 或 cw"));
            return;
        }

        struct RegionRange { double start = 0.0; double end = 0.0; };
        std::vector<RegionRange> ranges;
        const QStringList rangeParts = m_manualRegionRanges.split(';', Qt::SkipEmptyParts);
        for (QString part : rangeParts) {
            part.remove('[');
            part.remove(']');
            const QStringList values = part.split(QRegularExpression(QStringLiteral("[,\\s]+")),
                                                  Qt::SkipEmptyParts);
            if (values.size() != 2)
                continue;
            bool startOk = false;
            bool endOk = false;
            const double start = values[0].toDouble(&startOk);
            const double end = values[1].toDouble(&endOk);
            if (startOk && endOk)
                ranges.push_back({start, end});
        }
        if (ranges.empty()) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("手动分区角度格式无效"));
            return;
        }

        const bool directionReversed =
            (m_angleViewDirection == QStringLiteral("-a")) !=
            (m_angleIncreaseDirection == QStringLiteral("cw"));
        auto transformAngle = [this](double angle) {
            angle = wrapTo360(angle);
            if (m_angleViewDirection == QStringLiteral("-a"))
                angle = wrapTo360(-angle);
            if (m_angleIncreaseDirection == QStringLiteral("cw"))
                angle = wrapTo360(-angle);
            return wrapTo360(angle + m_manualZeroOffsetDeg);
        };
        for (auto &range : ranges) {
            const double transformedStart = transformAngle(range.start);
            const double transformedEnd = transformAngle(range.end);
            range = directionReversed
                ? RegionRange{transformedEnd, transformedStart}
                : RegionRange{transformedStart, transformedEnd};
        }

        auto intervalContains = [](double angle, const RegionRange &range) {
            angle = wrapTo360(angle);
            const double start = wrapTo360(range.start);
            const double end = wrapTo360(range.end);
            return start <= end ? (angle >= start && angle < end)
                                : (angle >= start || angle < end);
        };
        auto assignRegion = [&](double angle) {
            for (std::size_t i = 0; i < ranges.size(); ++i)
                if (intervalContains(angle, ranges[i]))
                    return static_cast<int>(i + 1);
            int nearest = 0;
            double nearestDistance = std::numeric_limits<double>::max();
            for (std::size_t i = 0; i < ranges.size(); ++i) {
                const double center = wrapTo360(ranges[i].start +
                    wrapTo360(ranges[i].end - ranges[i].start) * 0.5);
                const double distance = std::abs(wrapTo180(angle - center));
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearest = static_cast<int>(i);
                }
            }
            return nearest + 1;
        };

        struct LayerInput {
            int sliceId = 0;
            bool closed = false;
            std::vector<AxisVec3> worldPoints;
        };
        vtkIntArray *sliceIds = vtkIntArray::SafeDownCast(
            m_fittedContours->GetCellData()->GetArray("SliceIndex"));
        vtkIntArray *closedFlags = vtkIntArray::SafeDownCast(
            m_fittedContours->GetCellData()->GetArray("ClosedLayer"));
        std::map<int, LayerInput> layerMap;
        for (vtkIdType cellId = 0; cellId < m_fittedContours->GetNumberOfCells(); ++cellId) {
            vtkCell *cell = m_fittedContours->GetCell(cellId);
            if (!cell || cell->GetNumberOfPoints() < 4)
                continue;
            const int sliceId = sliceIds && sliceIds->GetNumberOfTuples() > cellId
                ? sliceIds->GetValue(cellId) : static_cast<int>(cellId);
            std::vector<AxisVec3> points;
            points.reserve(static_cast<std::size_t>(cell->GetNumberOfPoints()));
            for (vtkIdType i = 0; i < cell->GetNumberOfPoints(); ++i) {
                double p[3];
                m_fittedContours->GetPoint(cell->GetPointId(i), p);
                points.push_back(makeVec(p[0], p[1], p[2]));
            }
            auto &layer = layerMap[sliceId];
            if (points.size() > layer.worldPoints.size()) {
                layer.sliceId = sliceId;
                layer.closed = closedFlags && closedFlags->GetNumberOfTuples() > cellId
                    ? closedFlags->GetValue(cellId) != 0 : false;
                layer.worldPoints = std::move(points);
            }
        }
        if (layerMap.empty()) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("拟合轮廓中没有有效层"));
            return;
        }

        double axisStartRaw[3];
        double axisEndRaw[3];
        m_straightAxis->GetPoint(0, axisStartRaw);
        m_straightAxis->GetPoint(m_straightAxis->GetNumberOfPoints() - 1, axisEndRaw);
        const AxisVec3 axisStart = makeVec(axisStartRaw[0], axisStartRaw[1], axisStartRaw[2]);
        const AxisVec3 axisEnd = makeVec(axisEndRaw[0], axisEndRaw[1], axisEndRaw[2]);
        AxisVec3 axis = normalizedVec(subVec(axisEnd, axisStart));
        if (m_curvedAxis && m_curvedAxis->GetNumberOfPoints() >= 2) {
            double first[3];
            double last[3];
            m_curvedAxis->GetPoint(0, first);
            m_curvedAxis->GetPoint(m_curvedAxis->GetNumberOfPoints() - 1, last);
            if (dotVec(axis, subVec(makeVec(last[0], last[1], last[2]),
                                    makeVec(first[0], first[1], first[2]))) < 0.0)
                axis = mulVec(axis, -1.0);
        }
        const AxisVec3 origin = mulVec(addVec(axisStart, axisEnd), 0.5);
        const AxisVec3 reference = std::abs(axis.z) < 0.9 ? makeVec(0.0, 0.0, 1.0)
                                                          : makeVec(0.0, 1.0, 0.0);
        const AxisVec3 u = normalizedVec(crossVec(reference, axis));
        const AxisVec3 v = normalizedVec(crossVec(axis, u));
        auto worldToLocal = [&](const AxisVec3 &point) {
            const AxisVec3 relative = subVec(point, origin);
            return makeVec(dotVec(relative, u), dotVec(relative, v), dotVec(relative, axis));
        };
        auto localToWorld = [&](const AxisVec3 &point) {
            return addVec(origin, addVec(mulVec(u, point.x),
                addVec(mulVec(v, point.y), mulVec(axis, point.z))));
        };

        reportProgress(3);
        reportLog(QStringLiteral("开始生成等剂量路径：有效拟合层=%1，喷嘴长度=%2m，喷嘴外半径=%3m，安全间隙=%4m")
            .arg(layerMap.size()).arg(m_nozzleVerticalLength)
            .arg(m_sprayRodRadius).arg(m_safetyClearance));
        reportLog(QStringLiteral("分区=%1，零度偏移=%2度，观察方向=%3，角度方向=%4")
            .arg(ranges.size()).arg(m_manualZeroOffsetDeg)
            .arg(m_angleViewDirection, m_angleIncreaseDirection));

        vtkNew<vtkPoints> surfacePoints;
        vtkNew<vtkCellArray> surfacePolys;
        vtkNew<vtkIntArray> surfaceRegionIds;
        surfaceRegionIds->SetName("RegionId");
        vtkNew<vtkIntArray> surfaceSliceIds;
        surfaceSliceIds->SetName("SliceIndex");
        vtkNew<vtkUnsignedCharArray> surfaceColors;
        surfaceColors->SetName("Colors");
        surfaceColors->SetNumberOfComponents(3);

        vtkNew<vtkPoints> pathPoints;
        vtkNew<vtkCellArray> pathLines;
        vtkNew<vtkIntArray> pathSliceIds;
        pathSliceIds->SetName("SliceIndex");
        vtkNew<vtkIntArray> pathClosedFlags;
        pathClosedFlags->SetName("ClosedLayer");
        vtkNew<vtkIntArray> pathRegionIds;
        pathRegionIds->SetName("RegionId");

        std::vector<AxisVec3> anchors;
        int processedLayers = 0;
        int closedLayerCount = 0;
        double connectionAngle = m_connectionRefAngleDeg;
        double minimumRadialClearance = std::numeric_limits<double>::max();

        auto firstLayer = layerMap.begin();
        if (m_useTopOpenEndpoint && firstLayer != layerMap.end() && !firstLayer->second.closed) {
            const auto &points = firstLayer->second.worldPoints;
            const AxisVec3 local = worldToLocal(m_openEndpointMode == QStringLiteral("start")
                ? points.front() : points.back());
            connectionAngle = wrapTo360(std::atan2(local.y, local.x) * 180.0 / vtkMath::Pi());
        }

        for (const auto &entry : layerMap) {
            const LayerInput &layer = entry.second;
            std::vector<AxisVec3> localPoints;
            localPoints.reserve(layer.worldPoints.size());
            for (const auto &point : layer.worldPoints)
                localPoints.push_back(worldToLocal(point));
            if (layer.closed && localPoints.size() > 1 &&
                normVec(subVec(localPoints.front(), localPoints.back())) < 1e-10) {
                localPoints.pop_back();
            }
            if (localPoints.size() < 4)
                continue;

            std::vector<double> theta;
            std::vector<double> uValues;
            std::vector<double> vValues;
            if (layer.closed) {
                struct PolarPoint { double angle; AxisVec3 point; };
                std::vector<PolarPoint> polar;
                polar.reserve(localPoints.size());
                for (const auto &point : localPoints) {
                    polar.push_back({wrapTo360(std::atan2(point.y, point.x) *
                                              180.0 / vtkMath::Pi()), point});
                }
                std::sort(polar.begin(), polar.end(), [](const auto &left, const auto &right) {
                    return left.angle < right.angle;
                });
                for (const auto &item : polar) {
                    if (!theta.empty() && std::abs(item.angle - theta.back()) < 1e-8)
                        continue;
                    theta.push_back(item.angle);
                    uValues.push_back(item.point.x);
                    vValues.push_back(item.point.y);
                }
                if (theta.size() >= 4) {
                    theta.insert(theta.begin(), theta.back() - 360.0);
                    theta.push_back(theta[1] + 360.0);
                    uValues.insert(uValues.begin(), uValues.back());
                    uValues.push_back(uValues[1]);
                    vValues.insert(vValues.begin(), vValues.back());
                    vValues.push_back(vValues[1]);
                }
            } else {
                std::vector<std::pair<double, AxisVec3>> ordered;
                ordered.reserve(localPoints.size());
                double previous = std::atan2(localPoints.front().y, localPoints.front().x) *
                                  180.0 / vtkMath::Pi();
                ordered.push_back({previous, localPoints.front()});
                for (std::size_t i = 1; i < localPoints.size(); ++i) {
                    const double raw = std::atan2(localPoints[i].y, localPoints[i].x) *
                                       180.0 / vtkMath::Pi();
                    previous += wrapTo180(raw - previous);
                    ordered.push_back({previous, localPoints[i]});
                }
                if (ordered.back().first < ordered.front().first)
                    std::reverse(ordered.begin(), ordered.end());
                std::sort(ordered.begin(), ordered.end(), [](const auto &left, const auto &right) {
                    return left.first < right.first;
                });
                for (const auto &item : ordered) {
                    if (!theta.empty() && std::abs(item.first - theta.back()) < 1e-8)
                        continue;
                    theta.push_back(item.first);
                    uValues.push_back(item.second.x);
                    vValues.push_back(item.second.y);
                }
            }
            if (theta.size() < 4)
                continue;

            const int sampleCount = layer.closed ? m_closedSamples : m_openSamples;
            std::vector<double> queries;
            queries.reserve(static_cast<std::size_t>(sampleCount));
            if (layer.closed) {
                for (int i = 0; i < sampleCount; ++i)
                    queries.push_back(360.0 * i / sampleCount);
            } else {
                for (int i = 0; i < sampleCount; ++i) {
                    const double ratio = sampleCount > 1
                        ? static_cast<double>(i) / (sampleCount - 1) : 0.0;
                    queries.push_back(theta.front() + (theta.back() - theta.front()) * ratio);
                }
            }
            const std::vector<double> uSlopes = pchipSlopes(theta, uValues);
            const std::vector<double> vSlopes = pchipSlopes(theta, vValues);
            const double axialPosition = std::accumulate(localPoints.begin(), localPoints.end(), 0.0,
                [](double sum, const AxisVec3 &point) { return sum + point.z; }) /
                static_cast<double>(localPoints.size());

            std::vector<AxisVec3> referenceWorld;
            std::vector<AxisVec3> layerPathWorld;
            std::vector<int> regionIds;
            referenceWorld.reserve(queries.size());
            layerPathWorld.reserve(queries.size());
            regionIds.reserve(queries.size());
            for (double query : queries) {
                const double localU = pchipInterpolate(theta, uValues, uSlopes, query);
                const double localV = pchipInterpolate(theta, vValues, vSlopes, query);
                referenceWorld.push_back(localToWorld(makeVec(localU, localV, axialPosition)));
                const double radius = std::hypot(localU, localV);
                const double radialClearance = radius - m_sprayRodRadius;
                minimumRadialClearance = std::min(minimumRadialClearance, radialClearance);
                if (radialClearance + kClearanceComparisonToleranceM < m_safetyClearance) {
                    finish(nullptr, nullptr, nullptr, false,
                           QStringLiteral("第 %1 层喷嘴表面安全间隙不足：实际最小=%2m，要求=%3m，计算容差=%4m")
                               .arg(layer.sliceId + 1)
                               .arg(radialClearance, 0, 'f', 5)
                               .arg(m_safetyClearance, 0, 'f', 5)
                               .arg(kClearanceComparisonToleranceM, 0, 'f', 5));
                    return;
                }
                const AxisVec3 pathLocal = radius < 1e-12
                    ? makeVec(0.0, 0.0, axialPosition)
                    : makeVec(m_sprayRodRadius * localU / radius,
                              m_sprayRodRadius * localV / radius,
                              axialPosition);
                layerPathWorld.push_back(localToWorld(pathLocal));
                regionIds.push_back(assignRegion(wrapTo360(query)));
            }

            const vtkIdType surfaceStart = surfacePoints->GetNumberOfPoints();
            for (const auto &point : referenceWorld) {
                const AxisVec3 top = addVec(point, mulVec(axis, m_nozzleVerticalLength * 0.5));
                surfacePoints->InsertNextPoint(top.x, top.y, top.z);
            }
            for (const auto &point : referenceWorld) {
                const AxisVec3 bottom = subVec(point, mulVec(axis, m_nozzleVerticalLength * 0.5));
                surfacePoints->InsertNextPoint(bottom.x, bottom.y, bottom.z);
            }
            const vtkIdType pointCount = static_cast<vtkIdType>(referenceWorld.size());
            const vtkIdType segmentCount = layer.closed ? pointCount : pointCount - 1;
            for (vtkIdType i = 0; i < segmentCount; ++i) {
                const vtkIdType next = (i + 1) % pointCount;
                const vtkIdType ids[4] = {
                    surfaceStart + i, surfaceStart + next,
                    surfaceStart + pointCount + next, surfaceStart + pointCount + i
                };
                surfacePolys->InsertNextCell(4, ids);
                const int regionId = regionIds[static_cast<std::size_t>(i)];
                surfaceRegionIds->InsertNextValue(regionId);
                surfaceSliceIds->InsertNextValue(layer.sliceId);
                const auto color = matlabLinesColor(regionId);
                surfaceColors->InsertNextTypedTuple(color.data());
            }

            std::vector<vtkIdType> pathIds;
            pathIds.reserve(layerPathWorld.size() + (layer.closed ? 1 : 0));
            for (std::size_t i = 0; i < layerPathWorld.size(); ++i) {
                const auto &point = layerPathWorld[i];
                pathIds.push_back(pathPoints->InsertNextPoint(point.x, point.y, point.z));
                pathRegionIds->InsertNextValue(regionIds[i]);
            }
            if (layer.closed)
                pathIds.push_back(pathIds.front());
            pathLines->InsertNextCell(static_cast<vtkIdType>(pathIds.size()), pathIds.data());
            pathSliceIds->InsertNextValue(layer.sliceId);
            pathClosedFlags->InsertNextValue(layer.closed ? 1 : 0);

            std::size_t anchorIndex = 0;
            if (processedLayers == 0 && !layer.closed && m_useTopOpenEndpoint) {
                anchorIndex = m_openEndpointMode == QStringLiteral("start")
                    ? 0 : layerPathWorld.size() - 1;
            } else {
                double nearestDistance = std::numeric_limits<double>::max();
                for (std::size_t i = 0; i < queries.size(); ++i) {
                    const double distance = std::abs(wrapTo180(wrapTo360(queries[i]) - connectionAngle));
                    if (distance < nearestDistance) {
                        nearestDistance = distance;
                        anchorIndex = i;
                    }
                }
            }
            anchors.push_back(layerPathWorld[anchorIndex]);
            ++processedLayers;
            if (layer.closed)
                ++closedLayerCount;
            reportProgress(8 + static_cast<int>(72.0 * processedLayers / layerMap.size()));
            reportLog(QStringLiteral("第 %1 层：%2，路径点=%3，等剂量面片=%4")
                .arg(layer.sliceId + 1)
                .arg(layer.closed ? QStringLiteral("闭合") : QStringLiteral("开口"))
                .arg(layerPathWorld.size()).arg(segmentCount));
        }

        if (processedLayers == 0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("没有有效层生成等剂量路径"));
            return;
        }

        vtkSmartPointer<vtkPolyData> surface = vtkSmartPointer<vtkPolyData>::New();
        surface->SetPoints(surfacePoints);
        surface->SetPolys(surfacePolys);
        surface->GetCellData()->AddArray(surfaceRegionIds);
        surface->GetCellData()->AddArray(surfaceSliceIds);
        surface->GetCellData()->SetScalars(surfaceColors);

        vtkSmartPointer<vtkPolyData> path = vtkSmartPointer<vtkPolyData>::New();
        path->SetPoints(pathPoints);
        path->SetLines(pathLines);
        path->GetCellData()->AddArray(pathSliceIds);
        path->GetCellData()->AddArray(pathClosedFlags);
        path->GetPointData()->AddArray(pathRegionIds);

        vtkNew<vtkPoints> connectionPoints;
        vtkNew<vtkCellArray> connectionLines;
        for (std::size_t i = 0; i + 1 < anchors.size(); ++i) {
            const vtkIdType first = connectionPoints->InsertNextPoint(
                anchors[i].x, anchors[i].y, anchors[i].z);
            const vtkIdType second = connectionPoints->InsertNextPoint(
                anchors[i + 1].x, anchors[i + 1].y, anchors[i + 1].z);
            const vtkIdType ids[2] = {first, second};
            connectionLines->InsertNextCell(2, ids);
        }
        vtkSmartPointer<vtkPolyData> connections = vtkSmartPointer<vtkPolyData>::New();
        connections->SetPoints(connectionPoints);
        connections->SetLines(connectionLines);

        reportProgress(100);
        reportLog(QStringLiteral("等剂量路径完成：有效层=%1，闭合层=%2，开口层=%3，层间连接=%4")
            .arg(processedLayers).arg(closedLayerCount)
            .arg(processedLayers - closedLayerCount).arg(connections->GetNumberOfLines()));
        reportLog(QStringLiteral("喷嘴表面最小径向安全间隙=%1m，要求不少于=%2m，计算容差=%3m")
            .arg(minimumRadialClearance, 0, 'f', 5)
            .arg(m_safetyClearance, 0, 'f', 5)
            .arg(kClearanceComparisonToleranceM, 0, 'f', 5));
        finish(surface, path, connections, true,
               QStringLiteral("等剂量面与喷杆路径生成完成"));
    } catch (const std::exception &error) {
        finish(nullptr, nullptr, nullptr, false,
               QStringLiteral("等剂量路径异常: %1").arg(error.what()));
    } catch (...) {
        finish(nullptr, nullptr, nullptr, false,
               QStringLiteral("等剂量路径发生未知异常"));
    }
}

ContinuousPathTask::ContinuousPathTask(const QString &sourceName,
                                       vtkSmartPointer<vtkPolyData> sprayPath,
                                       vtkSmartPointer<vtkPolyData> straightAxis,
                                       double maxJoint6SweepDeg,
                                       double minPointSpacing,
                                       double maxTransitionDistance,
                                       bool reverseLayerOrder,
                                       bool autoReverseOpenLayers,
                                       QObject *receiver)
    : m_sourceName(sourceName),
      m_sprayPath(sprayPath),
      m_straightAxis(straightAxis),
      m_maxJoint6SweepDeg(std::clamp(maxJoint6SweepDeg, 1.0, 180.0)),
      m_minPointSpacing(std::max(0.0, minPointSpacing)),
      m_maxTransitionDistance(std::max(0.0, maxTransitionDistance)),
      m_reverseLayerOrder(reverseLayerOrder),
      m_autoReverseOpenLayers(autoReverseOpenLayers),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void ContinuousPathTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onContinuousPathProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(int, progress));
}

void ContinuousPathTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onContinuousPathLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(QString, message));
}

void ContinuousPathTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> path,
                         vtkSmartPointer<vtkPolyData> transitions,
                         vtkSmartPointer<vtkPolyData> resetMarkers,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onContinuousPathFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, path),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, transitions),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, resetMarkers),
                                  Q_ARG(bool, ok), Q_ARG(QString, message));
    };

    try {
        if (!m_sprayPath || m_sprayPath->GetNumberOfLines() == 0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("缺少等剂量喷杆路径"));
            return;
        }
        if (!m_straightAxis || m_straightAxis->GetNumberOfPoints() < 2) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("缺少直参考轴"));
            return;
        }

        struct LayerInput {
            int sliceId = 0;
            bool closed = false;
            std::vector<AxisVec3> points;
        };
        vtkIntArray *sliceIds = vtkIntArray::SafeDownCast(
            m_sprayPath->GetCellData()->GetArray("SliceIndex"));
        vtkIntArray *closedFlags = vtkIntArray::SafeDownCast(
            m_sprayPath->GetCellData()->GetArray("ClosedLayer"));
        std::map<int, LayerInput> layerMap;
        for (vtkIdType cellId = 0; cellId < m_sprayPath->GetNumberOfCells(); ++cellId) {
            vtkCell *cell = m_sprayPath->GetCell(cellId);
            if (!cell || cell->GetNumberOfPoints() < 2)
                continue;
            const int sliceId = sliceIds && sliceIds->GetNumberOfTuples() > cellId
                ? sliceIds->GetValue(cellId) : static_cast<int>(cellId);
            std::vector<AxisVec3> points;
            points.reserve(static_cast<std::size_t>(cell->GetNumberOfPoints()));
            for (vtkIdType i = 0; i < cell->GetNumberOfPoints(); ++i) {
                double point[3];
                m_sprayPath->GetPoint(cell->GetPointId(i), point);
                points.push_back(makeVec(point[0], point[1], point[2]));
            }
            auto &layer = layerMap[sliceId];
            if (points.size() > layer.points.size()) {
                layer.sliceId = sliceId;
                layer.closed = closedFlags && closedFlags->GetNumberOfTuples() > cellId
                    ? closedFlags->GetValue(cellId) != 0 : false;
                layer.points = std::move(points);
            }
        }
        if (layerMap.empty()) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("喷杆路径中没有有效层"));
            return;
        }

        double axisStartRaw[3];
        double axisEndRaw[3];
        m_straightAxis->GetPoint(0, axisStartRaw);
        m_straightAxis->GetPoint(m_straightAxis->GetNumberOfPoints() - 1, axisEndRaw);
        const AxisVec3 axisStart = makeVec(axisStartRaw[0], axisStartRaw[1], axisStartRaw[2]);
        const AxisVec3 axisEnd = makeVec(axisEndRaw[0], axisEndRaw[1], axisEndRaw[2]);
        const AxisVec3 axis = normalizedVec(subVec(axisEnd, axisStart));
        if (normVec(axis) < 1e-12) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("直参考轴长度无效"));
            return;
        }
        const AxisVec3 origin = mulVec(addVec(axisStart, axisEnd), 0.5);
        const AxisVec3 reference = std::abs(axis.z) < 0.9 ? makeVec(0.0, 0.0, 1.0)
                                                          : makeVec(0.0, 1.0, 0.0);
        const AxisVec3 u = normalizedVec(crossVec(reference, axis));
        const AxisVec3 v = normalizedVec(crossVec(axis, u));
        auto polarAngle = [&](const AxisVec3 &point) {
            const AxisVec3 relative = subVec(point, origin);
            return std::atan2(dotVec(relative, v), dotVec(relative, u)) *
                   180.0 / vtkMath::Pi();
        };

        std::vector<LayerInput> layers;
        layers.reserve(layerMap.size());
        for (auto &entry : layerMap)
            layers.push_back(std::move(entry.second));
        if (m_reverseLayerOrder)
            std::reverse(layers.begin(), layers.end());

        vtkNew<vtkPoints> pathPoints;
        vtkNew<vtkCellArray> pathLines;
        vtkNew<vtkIntArray> segmentIds;
        segmentIds->SetName("SegmentId");
        vtkNew<vtkIntArray> pathSliceIds;
        pathSliceIds->SetName("SliceIndex");
        vtkNew<vtkIntArray> pathClosedFlags;
        pathClosedFlags->SetName("ClosedLayer");
        vtkNew<vtkDoubleArray> segmentSweeps;
        segmentSweeps->SetName("Joint6SweepDeg");

        vtkNew<vtkPoints> transitionPoints;
        vtkNew<vtkCellArray> transitionLines;
        vtkNew<vtkIntArray> transitionFromSlices;
        transitionFromSlices->SetName("FromSliceIndex");
        vtkNew<vtkIntArray> transitionToSlices;
        transitionToSlices->SetName("ToSliceIndex");
        vtkNew<vtkIntArray> transitionWarnings;
        transitionWarnings->SetName("IsAnomalous");
        vtkNew<vtkDoubleArray> transitionDistances;
        transitionDistances->SetName("Distance");
        vtkNew<vtkUnsignedCharArray> transitionColors;
        transitionColors->SetName("Colors");
        transitionColors->SetNumberOfComponents(3);

        vtkNew<vtkPoints> resetPoints;
        vtkNew<vtkCellArray> resetVertices;
        vtkNew<vtkIntArray> resetSliceIds;
        resetSliceIds->SetName("SliceIndex");
        vtkNew<vtkIntArray> resetAfterSegments;
        resetAfterSegments->SetName("ResetAfterSegmentId");
        vtkNew<vtkUnsignedCharArray> resetColors;
        resetColors->SetName("Colors");
        resetColors->SetNumberOfComponents(3);

        int segmentId = 0;
        int resetCount = 0;
        int anomalousTransitionCount = 0;
        int processedLayers = 0;
        double totalTreatmentLength = 0.0;
        double totalTransitionLength = 0.0;
        double maxPointSpacing = 0.0;
        double maxTransitionLength = 0.0;
        AxisVec3 previousLayerEnd;
        int previousSliceId = -1;
        bool hasPreviousLayer = false;

        auto appendSegment = [&](const std::vector<AxisVec3> &points,
                                 int sliceId,
                                 bool closed,
                                 double sweep) {
            if (points.size() < 2)
                return false;
            std::vector<vtkIdType> ids;
            ids.reserve(points.size());
            for (const auto &point : points)
                ids.push_back(pathPoints->InsertNextPoint(point.x, point.y, point.z));
            pathLines->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
            segmentIds->InsertNextValue(segmentId);
            pathSliceIds->InsertNextValue(sliceId);
            pathClosedFlags->InsertNextValue(closed ? 1 : 0);
            segmentSweeps->InsertNextValue(sweep);
            for (std::size_t i = 1; i < points.size(); ++i) {
                const double spacing = normVec(subVec(points[i], points[i - 1]));
                totalTreatmentLength += spacing;
                maxPointSpacing = std::max(maxPointSpacing, spacing);
            }
            ++segmentId;
            return true;
        };

        auto appendResetMarker = [&](const AxisVec3 &point, int sliceId) {
            const vtkIdType id = resetPoints->InsertNextPoint(point.x, point.y, point.z);
            resetVertices->InsertNextCell(1, &id);
            resetSliceIds->InsertNextValue(sliceId);
            resetAfterSegments->InsertNextValue(segmentId - 1);
            const unsigned char white[3] = {255, 255, 255};
            resetColors->InsertNextTypedTuple(white);
            ++resetCount;
        };

        reportProgress(3);
        reportLog(QStringLiteral("开始生成分段连续路径：有效层=%1，关节6单段限制=%2度")
            .arg(layers.size()).arg(m_maxJoint6SweepDeg, 0, 'f', 1));
        reportLog(QStringLiteral("注意：当前按局部周向角进行保守分段，实际关节6角度仍需机械臂逆解复核"));

        for (auto &layer : layers) {
            std::vector<AxisVec3> points = std::move(layer.points);
            const double duplicateTolerance = std::max(1e-9, m_minPointSpacing * 0.1);
            if (layer.closed && points.size() > 1 &&
                normVec(subVec(points.front(), points.back())) <= duplicateTolerance) {
                points.pop_back();
            }

            std::vector<AxisVec3> filtered;
            filtered.reserve(points.size());
            for (const auto &point : points) {
                if (filtered.empty() ||
                    normVec(subVec(point, filtered.back())) >= m_minPointSpacing) {
                    filtered.push_back(point);
                }
            }
            if (!layer.closed && !points.empty() && !filtered.empty() &&
                normVec(subVec(points.back(), filtered.back())) > 1e-12) {
                if (filtered.size() == 1 ||
                    normVec(subVec(points.back(), filtered[filtered.size() - 2])) >= m_minPointSpacing) {
                    filtered.push_back(points.back());
                } else {
                    filtered.back() = points.back();
                }
            }
            while (layer.closed && filtered.size() > 2 &&
                   normVec(subVec(filtered.back(), filtered.front())) < m_minPointSpacing) {
                filtered.pop_back();
            }
            points = std::move(filtered);
            if (points.size() < 2)
                continue;

            if (layer.closed && hasPreviousLayer) {
                auto nearest = std::min_element(points.begin(), points.end(),
                    [&](const AxisVec3 &left, const AxisVec3 &right) {
                        return normVec(subVec(left, previousLayerEnd)) <
                               normVec(subVec(right, previousLayerEnd));
                    });
                std::rotate(points.begin(), nearest, points.end());
            } else if (!layer.closed && m_autoReverseOpenLayers && hasPreviousLayer &&
                       normVec(subVec(points.back(), previousLayerEnd)) <
                       normVec(subVec(points.front(), previousLayerEnd))) {
                std::reverse(points.begin(), points.end());
            }
            if (layer.closed)
                points.push_back(points.front());

            if (hasPreviousLayer) {
                const double distance = normVec(subVec(points.front(), previousLayerEnd));
                const vtkIdType first = transitionPoints->InsertNextPoint(
                    previousLayerEnd.x, previousLayerEnd.y, previousLayerEnd.z);
                const vtkIdType second = transitionPoints->InsertNextPoint(
                    points.front().x, points.front().y, points.front().z);
                const vtkIdType ids[2] = {first, second};
                transitionLines->InsertNextCell(2, ids);
                transitionFromSlices->InsertNextValue(previousSliceId);
                transitionToSlices->InsertNextValue(layer.sliceId);
                const bool anomalous = m_maxTransitionDistance > 0.0 &&
                                       distance > m_maxTransitionDistance;
                transitionWarnings->InsertNextValue(anomalous ? 1 : 0);
                transitionDistances->InsertNextValue(distance);
                const unsigned char normalColor[3] = {255, 193, 7};
                const unsigned char warningColor[3] = {255, 48, 48};
                transitionColors->InsertNextTypedTuple(anomalous ? warningColor : normalColor);
                totalTransitionLength += distance;
                maxTransitionLength = std::max(maxTransitionLength, distance);
                if (anomalous)
                    ++anomalousTransitionCount;
            }

            std::vector<double> angles;
            angles.reserve(points.size());
            angles.push_back(polarAngle(points.front()));
            for (std::size_t i = 1; i < points.size(); ++i) {
                const double raw = polarAngle(points[i]);
                angles.push_back(angles.back() + wrapTo180(raw - angles.back()));
            }

            int layerSegmentCount = 0;
            double layerMaxSweep = 0.0;
            std::vector<AxisVec3> currentSegment{points.front()};
            double currentSweep = 0.0;
            for (std::size_t i = 1; i < points.size(); ++i) {
                const AxisVec3 edgeStart = points[i - 1];
                const AxisVec3 edgeEnd = points[i];
                const double edgeSweep = std::abs(angles[i] - angles[i - 1]);
                double consumedSweep = 0.0;
                if (edgeSweep <= 1e-10) {
                    if (normVec(subVec(edgeEnd, currentSegment.back())) > 1e-12)
                        currentSegment.push_back(edgeEnd);
                    continue;
                }

                while (consumedSweep < edgeSweep - 1e-10) {
                    const double allowance = m_maxJoint6SweepDeg - currentSweep;
                    const double take = std::min(std::max(allowance, 0.0),
                                                 edgeSweep - consumedSweep);
                    if (take <= 1e-10) {
                        if (appendSegment(currentSegment, layer.sliceId, layer.closed, currentSweep)) {
                            ++layerSegmentCount;
                            layerMaxSweep = std::max(layerMaxSweep, currentSweep);
                            appendResetMarker(currentSegment.back(), layer.sliceId);
                        }
                        currentSegment.assign(1, currentSegment.back());
                        currentSweep = 0.0;
                        continue;
                    }

                    consumedSweep += take;
                    const double ratio = consumedSweep / edgeSweep;
                    const AxisVec3 boundary = addVec(edgeStart,
                        mulVec(subVec(edgeEnd, edgeStart), ratio));
                    if (normVec(subVec(boundary, currentSegment.back())) > 1e-12)
                        currentSegment.push_back(boundary);
                    currentSweep += take;

                    const bool hasMorePath = consumedSweep < edgeSweep - 1e-10 ||
                                             i + 1 < points.size();
                    if (currentSweep >= m_maxJoint6SweepDeg - 1e-9 && hasMorePath) {
                        if (appendSegment(currentSegment, layer.sliceId, layer.closed, currentSweep)) {
                            ++layerSegmentCount;
                            layerMaxSweep = std::max(layerMaxSweep, currentSweep);
                            appendResetMarker(currentSegment.back(), layer.sliceId);
                        }
                        currentSegment.assign(1, currentSegment.back());
                        currentSweep = 0.0;
                    }
                }
            }
            if (appendSegment(currentSegment, layer.sliceId, layer.closed, currentSweep)) {
                ++layerSegmentCount;
                layerMaxSweep = std::max(layerMaxSweep, currentSweep);
            }

            previousLayerEnd = points.back();
            previousSliceId = layer.sliceId;
            hasPreviousLayer = true;
            ++processedLayers;
            reportProgress(8 + static_cast<int>(84.0 * processedLayers / layers.size()));
            reportLog(QStringLiteral("第 %1 层：%2，执行段=%3，最大单段周向转角=%4度")
                .arg(layer.sliceId + 1)
                .arg(layer.closed ? QStringLiteral("闭合") : QStringLiteral("开口"))
                .arg(layerSegmentCount).arg(layerMaxSweep, 0, 'f', 2));
        }

        if (segmentId == 0) {
            finish(nullptr, nullptr, nullptr, false, QStringLiteral("没有生成有效的连续路径段"));
            return;
        }

        vtkSmartPointer<vtkPolyData> path = vtkSmartPointer<vtkPolyData>::New();
        path->SetPoints(pathPoints);
        path->SetLines(pathLines);
        path->GetCellData()->AddArray(segmentIds);
        path->GetCellData()->AddArray(pathSliceIds);
        path->GetCellData()->AddArray(pathClosedFlags);
        path->GetCellData()->AddArray(segmentSweeps);

        vtkSmartPointer<vtkPolyData> transitions = vtkSmartPointer<vtkPolyData>::New();
        transitions->SetPoints(transitionPoints);
        transitions->SetLines(transitionLines);
        transitions->GetCellData()->AddArray(transitionFromSlices);
        transitions->GetCellData()->AddArray(transitionToSlices);
        transitions->GetCellData()->AddArray(transitionWarnings);
        transitions->GetCellData()->AddArray(transitionDistances);
        transitions->GetCellData()->SetScalars(transitionColors);

        vtkSmartPointer<vtkPolyData> resetMarkers = vtkSmartPointer<vtkPolyData>::New();
        resetMarkers->SetPoints(resetPoints);
        resetMarkers->SetVerts(resetVertices);
        resetMarkers->GetPointData()->AddArray(resetSliceIds);
        resetMarkers->GetPointData()->AddArray(resetAfterSegments);
        resetMarkers->GetPointData()->SetScalars(resetColors);

        reportProgress(100);
        reportLog(QStringLiteral("分段连续路径完成：执行段=%1，关节6复位点=%2，处理长度=%3m，层间移动=%4m")
            .arg(segmentId).arg(resetCount)
            .arg(totalTreatmentLength, 0, 'f', 4)
            .arg(totalTransitionLength, 0, 'f', 4));
        reportLog(QStringLiteral("质量检查：最大点间距=%1m，最大层间连接=%2m，异常连接=%3")
            .arg(maxPointSpacing, 0, 'f', 5)
            .arg(maxTransitionLength, 0, 'f', 4)
            .arg(anomalousTransitionCount));
        finish(path, transitions, resetMarkers, true,
               QStringLiteral("分段连续路径已生成；复位动作和实际关节6角度尚待逆解验证"));
    } catch (const std::exception &error) {
        finish(nullptr, nullptr, nullptr, false,
               QStringLiteral("分段连续路径异常: %1").arg(error.what()));
    } catch (...) {
        finish(nullptr, nullptr, nullptr, false,
               QStringLiteral("分段连续路径发生未知异常"));
    }
}

NozzlePoseTask::NozzlePoseTask(const QString &sourceName,
                               vtkSmartPointer<vtkPolyData> fittedContours,
                               vtkSmartPointer<vtkPolyData> rotationStartMarkers,
                               vtkSmartPointer<vtkPolyData> straightAxis,
                               vtkSmartPointer<vtkPolyData> curvedAxis,
                               bool reverseLayerOrder,
                               double motionTcpToRoundedTipAxial,
                               double entryTipStandoff,
                               QObject *receiver)
    : m_sourceName(sourceName),
      m_fittedContours(fittedContours),
      m_rotationStartMarkers(rotationStartMarkers),
      m_straightAxis(straightAxis),
      m_curvedAxis(curvedAxis),
      m_reverseLayerOrder(reverseLayerOrder),
      m_motionTcpToRoundedTipAxial(motionTcpToRoundedTipAxial),
      m_entryTipStandoff(entryTipStandoff),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void NozzlePoseTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onNozzlePoseProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(int, progress));
}

void NozzlePoseTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onNozzlePoseLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName), Q_ARG(QString, message));
}

void NozzlePoseTask::run()
{
    auto finish = [this](vtkSmartPointer<vtkPolyData> sequence,
                         vtkSmartPointer<vtkPolyData> preview,
                         bool ok,
                         const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onNozzlePoseFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, sequence),
                                  Q_ARG(vtkSmartPointer<vtkPolyData>, preview),
                                  Q_ARG(bool, ok), Q_ARG(QString, message));
    };

    try {
        if (!m_fittedContours || m_fittedContours->GetNumberOfLines() == 0) {
            finish(nullptr, nullptr, false, QStringLiteral("缺少拟合切片轮廓"));
            return;
        }
        if (!m_straightAxis || m_straightAxis->GetNumberOfPoints() < 2) {
            finish(nullptr, nullptr, false, QStringLiteral("缺少蓝色直参考轴"));
            return;
        }

        double axisStartRaw[3];
        double axisEndRaw[3];
        m_straightAxis->GetPoint(0, axisStartRaw);
        m_straightAxis->GetPoint(m_straightAxis->GetNumberOfPoints() - 1, axisEndRaw);
        const AxisVec3 axisStart = makeVec(axisStartRaw[0], axisStartRaw[1], axisStartRaw[2]);
        const AxisVec3 axisEnd = makeVec(axisEndRaw[0], axisEndRaw[1], axisEndRaw[2]);
        vtkDataArray *openingCenterArray = m_straightAxis->GetFieldData()
            ? m_straightAxis->GetFieldData()->GetArray("OpeningBoundaryCenterXYZ") : nullptr;
        if (!openingCenterArray || openingCenterArray->GetNumberOfTuples() != 1 ||
            openingCenterArray->GetNumberOfComponents() < 3) {
            finish(nullptr, nullptr, false, QStringLiteral("缺少物理开口中心诊断数据"));
            return;
        }
        const AxisVec3 detectedMouthCenter = makeVec(
            openingCenterArray->GetComponent(0, 0),
            openingCenterArray->GetComponent(0, 1),
            openingCenterArray->GetComponent(0, 2));
        AxisVec3 axis = normalizedVec(subVec(axisEnd, axisStart));
        if (normVec(axis) <= 1e-12) {
            finish(nullptr, nullptr, false, QStringLiteral("蓝色直参考轴长度无效"));
            return;
        }
        if (m_curvedAxis && m_curvedAxis->GetNumberOfPoints() >= 2) {
            double firstRaw[3];
            double lastRaw[3];
            m_curvedAxis->GetPoint(0, firstRaw);
            m_curvedAxis->GetPoint(m_curvedAxis->GetNumberOfPoints() - 1, lastRaw);
            const AxisVec3 hint = subVec(makeVec(lastRaw[0], lastRaw[1], lastRaw[2]),
                                         makeVec(firstRaw[0], firstRaw[1], firstRaw[2]));
            if (dotVec(axis, hint) < 0.0)
                axis = mulVec(axis, -1.0);
        }
        const AxisVec3 axisOrigin = mulVec(addVec(axisStart, axisEnd), 0.5);
        const double mouthAxial = dotVec(subVec(detectedMouthCenter, axisOrigin), axis);
        const AxisVec3 cavityMouthTip = addVec(axisOrigin, mulVec(axis, mouthAxial));
        const double mouthProjectionOffset = normVec(
            subVec(detectedMouthCenter, cavityMouthTip));
        const double axisMin = std::min(dotVec(subVec(axisStart, axisOrigin), axis),
                                        dotVec(subVec(axisEnd, axisOrigin), axis));
        const double axisMax = std::max(dotVec(subVec(axisStart, axisOrigin), axis),
                                        dotVec(subVec(axisEnd, axisOrigin), axis));
        const AxisVec3 reference = std::abs(axis.z) < 0.9 ? makeVec(0.0, 0.0, 1.0)
                                                          : makeVec(0.0, 1.0, 0.0);
        const AxisVec3 globalU = normalizedVec(crossVec(reference, axis));
        if (normVec(globalU) <= 1e-12) {
            finish(nullptr, nullptr, false, QStringLiteral("无法建立喷嘴位姿局部坐标系"));
            return;
        }

        struct LayerInput {
            int sliceId = 0;
            std::vector<AxisVec3> points;
            AxisVec3 startMarker;
            bool hasStartMarker = false;
            double inwardDistance = 0.0;
        };
        struct OpenLayerInput {
            int sliceId = 0;
            AxisVec3 centroid;
            double axial = 0.0;
        };
        std::map<int, LayerInput> layerMap;
        std::vector<OpenLayerInput> openLayers;
        vtkIntArray *sliceIds = vtkIntArray::SafeDownCast(
            m_fittedContours->GetCellData()->GetArray("SliceIndex"));
        vtkIntArray *closedFlags = vtkIntArray::SafeDownCast(
            m_fittedContours->GetCellData()->GetArray("ClosedLayer"));
        int openLayerCount = 0;
        for (vtkIdType cellId = 0; cellId < m_fittedContours->GetNumberOfCells(); ++cellId) {
            vtkCell *cell = m_fittedContours->GetCell(cellId);
            if (!cell || cell->GetNumberOfPoints() < 3)
                continue;
            bool closed = closedFlags && closedFlags->GetNumberOfTuples() > cellId
                ? closedFlags->GetValue(cellId) != 0 : false;
            if (!closed) {
                double first[3];
                double last[3];
                m_fittedContours->GetPoint(cell->GetPointId(0), first);
                m_fittedContours->GetPoint(cell->GetPointId(cell->GetNumberOfPoints() - 1), last);
                closed = vtkMath::Distance2BetweenPoints(first, last) <= 1e-12;
            }
            if (!closed) {
                AxisVec3 centroid;
                for (vtkIdType pointId = 0; pointId < cell->GetNumberOfPoints(); ++pointId) {
                    double raw[3];
                    m_fittedContours->GetPoint(cell->GetPointId(pointId), raw);
                    centroid = addVec(centroid, makeVec(raw[0], raw[1], raw[2]));
                }
                centroid = mulVec(centroid, 1.0 / cell->GetNumberOfPoints());
                const int sliceId = sliceIds && sliceIds->GetNumberOfTuples() > cellId
                    ? sliceIds->GetValue(cellId) : static_cast<int>(cellId);
                openLayers.push_back({sliceId, centroid,
                    dotVec(subVec(centroid, axisOrigin), axis)});
                ++openLayerCount;
                continue;
            }

            const int sliceId = sliceIds && sliceIds->GetNumberOfTuples() > cellId
                ? sliceIds->GetValue(cellId) : static_cast<int>(cellId);
            std::vector<AxisVec3> points;
            points.reserve(static_cast<std::size_t>(cell->GetNumberOfPoints()));
            for (vtkIdType pointId = 0; pointId < cell->GetNumberOfPoints(); ++pointId) {
                double raw[3];
                m_fittedContours->GetPoint(cell->GetPointId(pointId), raw);
                points.push_back(makeVec(raw[0], raw[1], raw[2]));
            }
            if (points.size() > 2 && normVec(subVec(points.front(), points.back())) <= 1e-9)
                points.pop_back();
            auto &layer = layerMap[sliceId];
            if (points.size() > layer.points.size()) {
                layer.sliceId = sliceId;
                layer.points = std::move(points);
            }
        }
        if (layerMap.empty()) {
            finish(nullptr, nullptr, false, QStringLiteral("没有闭合切片可生成左右半圈位姿"));
            return;
        }
        if (m_rotationStartMarkers && m_rotationStartMarkers->GetNumberOfPoints() > 0) {
            vtkIntArray *markerSliceIds = vtkIntArray::SafeDownCast(
                m_rotationStartMarkers->GetPointData()->GetArray("SliceIndex"));
            for (vtkIdType pointId = 0; pointId < m_rotationStartMarkers->GetNumberOfPoints(); ++pointId) {
                const int sliceId = markerSliceIds && markerSliceIds->GetNumberOfTuples() > pointId
                    ? markerSliceIds->GetValue(pointId) : static_cast<int>(pointId);
                auto it = layerMap.find(sliceId);
                if (it == layerMap.end())
                    continue;
                double raw[3];
                m_rotationStartMarkers->GetPoint(pointId, raw);
                it->second.startMarker = makeVec(raw[0], raw[1], raw[2]);
                it->second.hasStartMarker = true;
            }
        }

        std::vector<LayerInput> layers;
        layers.reserve(layerMap.size());
        for (auto &entry : layerMap)
            layers.push_back(std::move(entry.second));

        double closedAxialMean = 0.0;
        for (const auto &layer : layers) {
            AxisVec3 centroid;
            for (const auto &point : layer.points)
                centroid = addVec(centroid, point);
            centroid = mulVec(centroid, 1.0 / layer.points.size());
            closedAxialMean += dotVec(subVec(centroid, axisOrigin), axis);
        }
        closedAxialMean /= layers.size();
        double openAxialMean = mouthAxial;
        AxisVec3 outwardAxis = mulVec(axis, -1.0);
        OpenLayerInput boundaryEntry;
        const OpenLayerInput *entryLayer = nullptr;
        bool usedBoundaryEntryFallback = false;
        if (!openLayers.empty()) {
            openAxialMean = 0.0;
            for (const auto &layer : openLayers)
                openAxialMean += layer.axial;
            openAxialMean /= openLayers.size();
            if (std::abs(openAxialMean - closedAxialMean) <= 1e-6) {
                finish(nullptr, nullptr, false,
                       QStringLiteral("开口层与闭合层轴向位置无法区分"));
                return;
            }
            outwardAxis = openAxialMean > closedAxialMean
                ? axis : mulVec(axis, -1.0);
            entryLayer = &openLayers.front();
            for (const auto &candidate : openLayers) {
                if (dotVec(candidate.centroid, outwardAxis) >
                    dotVec(entryLayer->centroid, outwardAxis)) {
                    entryLayer = &candidate;
                }
            }
        } else {
            int outermostSliceId = layers.front().sliceId;
            double outermostAxial = std::numeric_limits<double>::max();
            for (const auto &layer : layers) {
                AxisVec3 centroid;
                for (const auto &point : layer.points)
                    centroid = addVec(centroid, point);
                centroid = mulVec(centroid, 1.0 / layer.points.size());
                const double axial = dotVec(subVec(centroid, axisOrigin), axis);
                if (axial < outermostAxial) {
                    outermostAxial = axial;
                    outermostSliceId = layer.sliceId;
                }
            }
            boundaryEntry = {outermostSliceId, cavityMouthTip, mouthAxial};
            entryLayer = &boundaryEntry;
            usedBoundaryEntryFallback = true;
            reportLog(QStringLiteral("入口几何诊断：全部 %1 层均为闭合喷涂层；使用术区开口中心在蓝轴上的投影作为入口，闭合层全部保留")
                .arg(layers.size()));
        }
        if (!std::isfinite(m_motionTcpToRoundedTipAxial) ||
            m_motionTcpToRoundedTipAxial < 0.0 ||
            m_motionTcpToRoundedTipAxial > 0.050) {
            finish(nullptr, nullptr, false,
                   QStringLiteral("运动 TCP 到圆头的轴向距离无效"));
            return;
        }
        if (!std::isfinite(m_entryTipStandoff) || m_entryTipStandoff < 0.0 ||
            m_entryTipStandoff > 0.150) {
            finish(nullptr, nullptr, false,
                   QStringLiteral("圆头入口外停距离无效"));
            return;
        }
        // Keep the rounded end outside the detected physical mouth. The motion TCP is
        // behind the rounded end by its configured lead.
        const AxisVec3 cavityEntryTcp = addVec(
            cavityMouthTip,
            mulVec(outwardAxis,
                   m_motionTcpToRoundedTipAxial + m_entryTipStandoff));
        constexpr double preEntryDistanceM = kEntryNormalApproachDistanceM;
        const AxisVec3 preEntryTcp = addVec(
            cavityEntryTcp, mulVec(outwardAxis, preEntryDistanceM));

        for (auto &layer : layers) {
            AxisVec3 centroid;
            for (const auto &point : layer.points)
                centroid = addVec(centroid, point);
            centroid = mulVec(centroid, 1.0 / layer.points.size());
            layer.inwardDistance = -dotVec(
                subVec(centroid, cavityEntryTcp), outwardAxis);
            if (!std::isfinite(layer.inwardDistance) ||
                layer.inwardDistance <= 0.0005) {
                finish(nullptr, nullptr, false,
                       QStringLiteral("闭合喷涂层 %1 不在入口内侧：轴向距离=%2mm")
                           .arg(layer.sliceId + 1)
                           .arg(layer.inwardDistance * 1000.0, 0, 'f', 3));
                return;
            }
        }
        std::sort(layers.begin(), layers.end(), [](const LayerInput &first,
                                                   const LayerInput &second) {
            if (std::abs(first.inwardDistance - second.inwardDistance) > 1e-9)
                return first.inwardDistance < second.inwardDistance;
            return first.sliceId < second.sliceId;
        });
        if (m_reverseLayerOrder)
            std::reverse(layers.begin(), layers.end());

        const auto depthRange = std::minmax_element(
            layers.begin(), layers.end(), [](const LayerInput &first,
                                             const LayerInput &second) {
                return first.inwardDistance < second.inwardDistance;
            });
        const double nearestLayerDistance = depthRange.first->inwardDistance;
        const double deepestLayerDistance = depthRange.second->inwardDistance;
        reportLog(QStringLiteral(
            "喷涂层序诊断：执行方向=%1，最近层距入口=%2mm，最深层距入口=%3mm，层数=%4")
            .arg(m_reverseLayerOrder ? QStringLiteral("底部到入口")
                                     : QStringLiteral("入口到底部"))
            .arg(nearestLayerDistance * 1000.0, 0, 'f', 2)
            .arg(deepestLayerDistance * 1000.0, 0, 'f', 2)
            .arg(layers.size()));

        reportLog(QStringLiteral("入口几何诊断：蓝轴起点=[%1,%2,%3]，终点=[%4,%5,%6]，开口层=%7，闭合层=%8")
            .arg(axisStart.x, 0, 'f', 6).arg(axisStart.y, 0, 'f', 6)
            .arg(axisStart.z, 0, 'f', 6).arg(axisEnd.x, 0, 'f', 6)
            .arg(axisEnd.y, 0, 'f', 6).arg(axisEnd.z, 0, 'f', 6)
            .arg(openLayers.size()).arg(layers.size()));
        reportLog(QStringLiteral("入口几何诊断：术区开口中心到蓝轴横向距离=%1mm，入口来源=%2")
            .arg(mouthProjectionOffset * 1000.0, 0, 'f', 3)
            .arg(usedBoundaryEntryFallback
                 ? QStringLiteral("开口中心投影（无开口切片回退）")
                 : QStringLiteral("最外开口切片")));
        reportLog(QStringLiteral("入口几何诊断：开口轴向均值=%1，闭合轴向均值=%2，入口层=%3，入口层质心=[%4,%5,%6]，入口TCP=[%7,%8,%9]，外向轴=[%10,%11,%12]")
            .arg(openAxialMean, 0, 'f', 6).arg(closedAxialMean, 0, 'f', 6)
            .arg(entryLayer->sliceId + 1)
            .arg(entryLayer->centroid.x, 0, 'f', 6)
            .arg(entryLayer->centroid.y, 0, 'f', 6)
            .arg(entryLayer->centroid.z, 0, 'f', 6)
            .arg(cavityEntryTcp.x, 0, 'f', 6)
            .arg(cavityEntryTcp.y, 0, 'f', 6)
            .arg(cavityEntryTcp.z, 0, 'f', 6)
            .arg(outwardAxis.x, 0, 'f', 6)
            .arg(outwardAxis.y, 0, 'f', 6)
            .arg(outwardAxis.z, 0, 'f', 6));

        vtkNew<vtkPoints> posePoints;
        vtkNew<vtkCellArray> poseVertices;
        vtkNew<vtkIntArray> sequenceIndices;
        sequenceIndices->SetName("SequenceIndex");
        vtkNew<vtkIntArray> poseSliceIds;
        poseSliceIds->SetName("SliceIndex");
        vtkNew<vtkIntArray> motionPhases;
        motionPhases->SetName("MotionPhase");
        vtkNew<vtkIntArray> plasmaEnabled;
        plasmaEnabled->SetName("PlasmaEnabled");
        vtkNew<vtkDoubleArray> rotations;
        rotations->SetName("Joint6GeometricDeg");
        vtkNew<vtkDoubleArray> nozzleDirections;
        nozzleDirections->SetName("NozzleDirection");
        nozzleDirections->SetNumberOfComponents(3);
        vtkNew<vtkDoubleArray> axisDirections;
        axisDirections->SetName("AxisDirection");
        axisDirections->SetNumberOfComponents(3);
        vtkNew<vtkDoubleArray> quaternions;
        quaternions->SetName("QuaternionXYZW");
        quaternions->SetNumberOfComponents(4);
        vtkNew<vtkDoubleArray> targetPoints;
        targetPoints->SetName("TargetPoint");
        targetPoints->SetNumberOfComponents(3);

        vtkNew<vtkPoints> previewPoints;
        vtkNew<vtkCellArray> previewVertices;
        vtkNew<vtkDoubleArray> previewVectors;
        previewVectors->SetName("PreviewVector");
        previewVectors->SetNumberOfComponents(3);
        vtkNew<vtkIntArray> previewSliceIds;
        previewSliceIds->SetName("SliceIndex");
        vtkNew<vtkDoubleArray> previewRotations;
        previewRotations->SetName("Joint6GeometricDeg");
        vtkNew<vtkUnsignedCharArray> previewColors;
        previewColors->SetName("Colors");
        previewColors->SetNumberOfComponents(3);

        auto quaternionFromFrame = [](const AxisVec3 &frameX,
                                      const AxisVec3 &frameY,
                                      const AxisVec3 &frameZ) {
            const double m00 = frameX.x, m01 = frameY.x, m02 = frameZ.x;
            const double m10 = frameX.y, m11 = frameY.y, m12 = frameZ.y;
            const double m20 = frameX.z, m21 = frameY.z, m22 = frameZ.z;
            double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
            const double trace = m00 + m11 + m22;
            if (trace > 0.0) {
                const double scale = 2.0 * std::sqrt(trace + 1.0);
                w = 0.25 * scale;
                x = (m21 - m12) / scale;
                y = (m02 - m20) / scale;
                z = (m10 - m01) / scale;
            } else if (m00 > m11 && m00 > m22) {
                const double scale = 2.0 * std::sqrt(std::max(0.0, 1.0 + m00 - m11 - m22));
                w = scale > 1e-12 ? (m21 - m12) / scale : 1.0;
                x = 0.25 * scale;
                y = scale > 1e-12 ? (m01 + m10) / scale : 0.0;
                z = scale > 1e-12 ? (m02 + m20) / scale : 0.0;
            } else if (m11 > m22) {
                const double scale = 2.0 * std::sqrt(std::max(0.0, 1.0 + m11 - m00 - m22));
                w = scale > 1e-12 ? (m02 - m20) / scale : 1.0;
                x = scale > 1e-12 ? (m01 + m10) / scale : 0.0;
                y = 0.25 * scale;
                z = scale > 1e-12 ? (m12 + m21) / scale : 0.0;
            } else {
                const double scale = 2.0 * std::sqrt(std::max(0.0, 1.0 + m22 - m00 - m11));
                w = scale > 1e-12 ? (m10 - m01) / scale : 1.0;
                x = scale > 1e-12 ? (m02 + m20) / scale : 0.0;
                y = scale > 1e-12 ? (m12 + m21) / scale : 0.0;
                z = 0.25 * scale;
            }
            const double norm = std::sqrt(x * x + y * y + z * z + w * w);
            if (norm > 1e-12) {
                x /= norm; y /= norm; z /= norm; w /= norm;
            }
            return std::array<double, 4>{x, y, z, w};
        };

        int sequenceIndex = 0;
        int processedLayers = 0;
        int fallbackMarkerCount = 0;
        constexpr double angularStepDeg = 5.0;
        reportProgress(3);
        reportLog(QStringLiteral("开始生成喷嘴离线位姿：闭合层=%1，开口层跳过=%2，角度步长=%3度")
            .arg(layers.size()).arg(openLayerCount).arg(angularStepDeg, 0, 'f', 1));

        for (const auto &layer : layers) {
            AxisVec3 centroid;
            for (const auto &point : layer.points)
                centroid = addVec(centroid, point);
            centroid = mulVec(centroid, 1.0 / layer.points.size());
            const double axial = std::clamp(dotVec(subVec(centroid, axisOrigin), axis),
                                            axisMin, axisMax);
            const AxisVec3 tcp = addVec(axisOrigin, mulVec(axis, axial));

            AxisVec3 zeroDirection = globalU;
            if (layer.hasStartMarker) {
                AxisVec3 markerDirection = subVec(layer.startMarker, tcp);
                markerDirection = subVec(markerDirection, mulVec(axis, dotVec(markerDirection, axis)));
                if (normVec(markerDirection) > 1e-12)
                    zeroDirection = normalizedVec(markerDirection);
            } else {
                ++fallbackMarkerCount;
            }
            const AxisVec3 positiveDirection = normalizedVec(crossVec(axis, zeroDirection));

            auto targetAtAngle = [&](double angleDeg) {
                const double angleRad = angleDeg * vtkMath::Pi() / 180.0;
                const AxisVec3 ray = addVec(mulVec(zeroDirection, std::cos(angleRad)),
                                            mulVec(positiveDirection, std::sin(angleRad)));
                const AxisVec3 side = addVec(mulVec(zeroDirection, -std::sin(angleRad)),
                                             mulVec(positiveDirection, std::cos(angleRad)));
                double bestDistance = std::numeric_limits<double>::max();
                AxisVec3 bestPoint;
                bool found = false;
                double fallbackAngle = std::numeric_limits<double>::max();
                AxisVec3 fallbackPoint = layer.points.front();
                for (std::size_t i = 0; i < layer.points.size(); ++i) {
                    const AxisVec3 first = layer.points[i];
                    const AxisVec3 second = layer.points[(i + 1) % layer.points.size()];
                    const AxisVec3 firstRel = subVec(first, tcp);
                    const AxisVec3 secondRel = subVec(second, tcp);
                    const double firstAlong = dotVec(firstRel, ray);
                    const double secondAlong = dotVec(secondRel, ray);
                    const double firstSide = dotVec(firstRel, side);
                    const double secondSide = dotVec(secondRel, side);

                    const double candidateAngle = std::abs(std::atan2(firstSide, firstAlong));
                    if (firstAlong > 0.0 && candidateAngle < fallbackAngle) {
                        fallbackAngle = candidateAngle;
                        fallbackPoint = first;
                    }

                    auto consider = [&](double ratio) {
                        if (ratio < 0.0 || ratio > 1.0)
                            return;
                        const double distance = firstAlong + ratio * (secondAlong - firstAlong);
                        if (distance <= 0.0 || distance >= bestDistance)
                            return;
                        bestDistance = distance;
                        bestPoint = addVec(first, mulVec(subVec(second, first), ratio));
                        found = true;
                    };
                    constexpr double epsilon = 1e-10;
                    if (std::abs(firstSide) <= epsilon)
                        consider(0.0);
                    const double denominator = secondSide - firstSide;
                    if (((firstSide < -epsilon && secondSide > epsilon) ||
                         (firstSide > epsilon && secondSide < -epsilon) ||
                         std::abs(secondSide) <= epsilon) &&
                        std::abs(denominator) > epsilon) {
                        consider(-firstSide / denominator);
                    }
                }
                return found ? bestPoint : fallbackPoint;
            };

            auto poseFrame = [&](const AxisVec3 &target) {
                AxisVec3 nozzle = subVec(target, tcp);
                nozzle = subVec(nozzle, mulVec(axis, dotVec(nozzle, axis)));
                nozzle = normalizedVec(nozzle);
                AxisVec3 frameX = subVec(axis, mulVec(nozzle, dotVec(axis, nozzle)));
                frameX = normalizedVec(frameX);
                AxisVec3 frameY = normalizedVec(crossVec(nozzle, frameX));
                return std::array<AxisVec3, 3>{frameX, frameY, nozzle};
            };

            auto appendPose = [&](double angleDeg, int phase, bool plasmaOn) {
                const AxisVec3 target = targetAtAngle(angleDeg);
                const auto frame = poseFrame(target);
                if (normVec(frame[0]) <= 1e-12 || normVec(frame[1]) <= 1e-12 ||
                    normVec(frame[2]) <= 1e-12) {
                    return;
                }
                const auto quaternion = quaternionFromFrame(frame[0], frame[1], frame[2]);
                const vtkIdType id = posePoints->InsertNextPoint(tcp.x, tcp.y, tcp.z);
                poseVertices->InsertNextCell(1, &id);
                sequenceIndices->InsertNextValue(sequenceIndex++);
                poseSliceIds->InsertNextValue(layer.sliceId);
                motionPhases->InsertNextValue(phase);
                plasmaEnabled->InsertNextValue(plasmaOn ? 1 : 0);
                rotations->InsertNextValue(angleDeg);
                const double nozzleTuple[3] = {frame[2].x, frame[2].y, frame[2].z};
                const double axisTuple[3] = {frame[0].x, frame[0].y, frame[0].z};
                const double targetTuple[3] = {target.x, target.y, target.z};
                nozzleDirections->InsertNextTuple(nozzleTuple);
                axisDirections->InsertNextTuple(axisTuple);
                quaternions->InsertNextTuple(quaternion.data());
                targetPoints->InsertNextTuple(targetTuple);
            };

            auto appendSweep = [&](double startDeg, double endDeg, int phase, bool plasmaOn) {
                const int steps = std::max(1, static_cast<int>(
                    std::ceil(std::abs(endDeg - startDeg) / angularStepDeg)));
                for (int step = 0; step <= steps; ++step) {
                    const double ratio = static_cast<double>(step) / steps;
                    appendPose(startDeg + ratio * (endDeg - startDeg), phase, plasmaOn);
                }
            };

            if (processedLayers > 0)
                appendPose(0.0, 4, false);
            appendSweep(0.0, 180.0, 0, true);
            appendSweep(180.0, 0.0, 1, false);
            appendSweep(0.0, -180.0, 2, true);
            appendSweep(-180.0, 0.0, 3, false);

            const double previewAngles[4] = {0.0, 90.0, 180.0, -90.0};
            for (double angleDeg : previewAngles) {
                const AxisVec3 target = targetAtAngle(angleDeg);
                AxisVec3 direction = subVec(target, tcp);
                direction = subVec(direction, mulVec(axis, dotVec(direction, axis)));
                const double radius = normVec(direction);
                if (radius <= 1e-12)
                    continue;
                direction = normalizedVec(direction);
                const double arrowLength = std::clamp(radius * 0.65, 0.004, 0.020);
                const AxisVec3 vector = mulVec(direction, arrowLength);
                const vtkIdType id = previewPoints->InsertNextPoint(tcp.x, tcp.y, tcp.z);
                previewVertices->InsertNextCell(1, &id);
                const double vectorTuple[3] = {vector.x, vector.y, vector.z};
                previewVectors->InsertNextTuple(vectorTuple);
                previewSliceIds->InsertNextValue(layer.sliceId);
                previewRotations->InsertNextValue(angleDeg);
                const unsigned char startColor[3] = {255, 64, 200};
                const unsigned char leftColor[3] = {0, 210, 175};
                const unsigned char rightColor[3] = {255, 145, 0};
                const unsigned char sharedEndColor[3] = {90, 170, 255};
                previewColors->InsertNextTypedTuple(
                    std::abs(angleDeg) < 1e-9 ? startColor
                    : std::abs(std::abs(angleDeg) - 180.0) < 1e-9 ? sharedEndColor
                    : angleDeg > 0.0 ? leftColor : rightColor);
            }

            ++processedLayers;
            reportProgress(5 + static_cast<int>(90.0 * processedLayers / layers.size()));
            reportLog(QStringLiteral("第 %1 层：距入口=%2mm，TCP 已投影到蓝轴，位姿序列=%3，执行顺序=左半圈/回零/右半圈/回零")
                .arg(layer.sliceId + 1)
                .arg(layer.inwardDistance * 1000.0, 0, 'f', 2)
                .arg(sequenceIndex));
        }

        auto appendEntryMarker = [&](const AxisVec3 &origin,
                                     const AxisVec3 &direction,
                                     const unsigned char color[3]) {
            const vtkIdType id = previewPoints->InsertNextPoint(origin.x, origin.y, origin.z);
            previewVertices->InsertNextCell(1, &id);
            const double vectorTuple[3] = {direction.x, direction.y, direction.z};
            previewVectors->InsertNextTuple(vectorTuple);
            previewSliceIds->InsertNextValue(entryLayer->sliceId);
            previewRotations->InsertNextValue(0.0);
            previewColors->InsertNextTypedTuple(color);
        };
        const unsigned char entryColor[3] = {255, 235, 0};
        const unsigned char preEntryColor[3] = {255, 255, 255};
        const unsigned char mouthTipColor[3] = {0, 220, 255};
        appendEntryMarker(cavityMouthTip, mulVec(outwardAxis, 0.030), mouthTipColor);
        appendEntryMarker(cavityEntryTcp, mulVec(outwardAxis, 0.030), entryColor);
        appendEntryMarker(preEntryTcp, mulVec(outwardAxis, -0.030), preEntryColor);

        if (sequenceIndex == 0) {
            finish(nullptr, nullptr, false, QStringLiteral("没有生成有效喷嘴位姿"));
            return;
        }

        vtkSmartPointer<vtkPolyData> sequence = vtkSmartPointer<vtkPolyData>::New();
        sequence->SetPoints(posePoints);
        sequence->SetVerts(poseVertices);
        sequence->GetPointData()->AddArray(sequenceIndices);
        sequence->GetPointData()->AddArray(poseSliceIds);
        sequence->GetPointData()->AddArray(motionPhases);
        sequence->GetPointData()->AddArray(plasmaEnabled);
        sequence->GetPointData()->AddArray(rotations);
        sequence->GetPointData()->SetVectors(nozzleDirections);
        sequence->GetPointData()->AddArray(axisDirections);
        sequence->GetPointData()->AddArray(quaternions);
        sequence->GetPointData()->AddArray(targetPoints);

        double firstQuaternion[4] = {0.0, 0.0, 0.0, 1.0};
        quaternions->GetTuple(0, firstQuaternion);
        vtkNew<vtkDoubleArray> cavityMouthTipPose;
        cavityMouthTipPose->SetName("CavityMouthTipPoseXYZXYZW");
        cavityMouthTipPose->SetNumberOfComponents(7);
        const double mouthTipTuple[7] = {
            cavityMouthTip.x, cavityMouthTip.y, cavityMouthTip.z,
            firstQuaternion[0], firstQuaternion[1], firstQuaternion[2], firstQuaternion[3]};
        cavityMouthTipPose->InsertNextTuple(mouthTipTuple);
        sequence->GetFieldData()->AddArray(cavityMouthTipPose);

        vtkNew<vtkDoubleArray> cavityEntryPose;
        cavityEntryPose->SetName("CavityEntryTcpPoseXYZXYZW");
        cavityEntryPose->SetNumberOfComponents(7);
        const double entryTuple[7] = {
            cavityEntryTcp.x, cavityEntryTcp.y, cavityEntryTcp.z,
            firstQuaternion[0], firstQuaternion[1], firstQuaternion[2], firstQuaternion[3]};
        cavityEntryPose->InsertNextTuple(entryTuple);
        sequence->GetFieldData()->AddArray(cavityEntryPose);

        vtkNew<vtkDoubleArray> preEntryPose;
        preEntryPose->SetName("PreEntryTcpPoseXYZXYZW");
        preEntryPose->SetNumberOfComponents(7);
        const double preEntryTuple[7] = {
            preEntryTcp.x, preEntryTcp.y, preEntryTcp.z,
            firstQuaternion[0], firstQuaternion[1], firstQuaternion[2], firstQuaternion[3]};
        preEntryPose->InsertNextTuple(preEntryTuple);
        sequence->GetFieldData()->AddArray(preEntryPose);

        vtkNew<vtkDoubleArray> cavityAxis;
        cavityAxis->SetName("CavityAxisOutward");
        cavityAxis->SetNumberOfComponents(3);
        const double axisTuple[3] = {outwardAxis.x, outwardAxis.y, outwardAxis.z};
        cavityAxis->InsertNextTuple(axisTuple);
        sequence->GetFieldData()->AddArray(cavityAxis);

        vtkNew<vtkIntArray> entrySliceIndex;
        entrySliceIndex->SetName("CavityEntrySliceIndex");
        entrySliceIndex->InsertNextValue(entryLayer->sliceId);
        sequence->GetFieldData()->AddArray(entrySliceIndex);

        vtkNew<vtkDoubleArray> preEntryDistance;
        preEntryDistance->SetName("PreEntryDistanceM");
        preEntryDistance->InsertNextValue(preEntryDistanceM);
        sequence->GetFieldData()->AddArray(preEntryDistance);

        vtkNew<vtkDoubleArray> entryTipStandoff;
        entryTipStandoff->SetName("EntryTipStandoffM");
        entryTipStandoff->InsertNextValue(m_entryTipStandoff);
        sequence->GetFieldData()->AddArray(entryTipStandoff);

        auto addVectorDiagnostic = [&sequence](const char *name, const AxisVec3 &value) {
            vtkNew<vtkDoubleArray> array;
            array->SetName(name);
            array->SetNumberOfComponents(3);
            const double tuple[3] = {value.x, value.y, value.z};
            array->InsertNextTuple(tuple);
            sequence->GetFieldData()->AddArray(array);
        };
        addVectorDiagnostic("StraightAxisStartXYZ", axisStart);
        addVectorDiagnostic("StraightAxisEndXYZ", axisEnd);
        addVectorDiagnostic("EntryLayerCentroidXYZ", entryLayer->centroid);
        vtkNew<vtkIntArray> openLayerDiagnostic;
        openLayerDiagnostic->SetName("OpenLayerCount");
        openLayerDiagnostic->InsertNextValue(static_cast<int>(openLayers.size()));
        sequence->GetFieldData()->AddArray(openLayerDiagnostic);
        vtkNew<vtkIntArray> closedLayerDiagnostic;
        closedLayerDiagnostic->SetName("ClosedLayerCount");
        closedLayerDiagnostic->InsertNextValue(static_cast<int>(layers.size()));
        sequence->GetFieldData()->AddArray(closedLayerDiagnostic);
        vtkNew<vtkDoubleArray> axialMeans;
        axialMeans->SetName("OpenClosedAxialMean");
        axialMeans->SetNumberOfComponents(2);
        const double axialMeanTuple[2] = {openAxialMean, closedAxialMean};
        axialMeans->InsertNextTuple(axialMeanTuple);
        sequence->GetFieldData()->AddArray(axialMeans);

        vtkSmartPointer<vtkPolyData> preview = vtkSmartPointer<vtkPolyData>::New();
        preview->SetPoints(previewPoints);
        preview->SetVerts(previewVertices);
        preview->GetPointData()->SetVectors(previewVectors);
        preview->GetPointData()->AddArray(previewSliceIds);
        preview->GetPointData()->AddArray(previewRotations);
        preview->GetPointData()->SetScalars(previewColors);

        reportProgress(100);
        reportLog(QStringLiteral("喷嘴位姿生成完成：闭合层=%1，位姿=%2，预览箭头=%3，缺失起点回退=%4")
            .arg(processedLayers).arg(sequence->GetNumberOfPoints())
            .arg(preview->GetNumberOfPoints()).arg(fallbackMarkerCount));
        reportLog(QStringLiteral("机械臂安全入口已生成：圆头停在物理开口外 %1mm，侧喷口TCP相对圆头后移=%2mm，来源=%3，参考层=%4，安全入口到预入口=%5mm，第一喷涂层距安全入口=%6mm，最深层距安全入口=%7mm")
            .arg(m_entryTipStandoff * 1000.0, 0, 'f', 1)
            .arg(m_motionTcpToRoundedTipAxial * 1000.0, 0, 'f', 1)
            .arg(usedBoundaryEntryFallback
                 ? QStringLiteral("术区开口中心投影")
                 : QStringLiteral("最外开口切片"))
            .arg(entryLayer->sliceId + 1)
            .arg(preEntryDistanceM * 1000.0, 0, 'f', 1)
            .arg(layers.front().inwardDistance * 1000.0, 0, 'f', 2)
            .arg(deepestLayerDistance * 1000.0, 0, 'f', 2));
        reportLog(QStringLiteral("入口标记：青色箭头根部=视觉物理开口，黄色箭头根部=安全入口侧喷口 TCP，白色箭头根部=预入口；安全入口处圆头仍在开口外，粗蓝线=腔体参考轴"));
        reportLog(QStringLiteral("位姿仅位于当前几何坐标系；尚未执行手眼标定、工具坐标变换、逆解或碰撞检查"));
        finish(sequence, preview, true, QStringLiteral("喷嘴离线几何位姿已生成"));
    } catch (const std::exception &error) {
        finish(nullptr, nullptr, false,
               QStringLiteral("喷嘴位姿异常: %1").arg(error.what()));
    } catch (...) {
        finish(nullptr, nullptr, false, QStringLiteral("喷嘴位姿发生未知异常"));
    }
}

// ==========================================
// CropTask 实现
// ==========================================

CropTask::CropTask(const CropParams &params, QObject *receiver)
    : m_params(params), m_receiver(receiver)
{
    // 设置自动删除，任务运行结束后自动释放内存
    setAutoDelete(true);
}

void CropTask::run()
{
    try {
        // --- 这里的逻辑与原 MainOpengl::DoCrop 中的核心算法完全一致 ---

        if (!m_params.inputCloud || !m_params.compositeMatrix)
        {
            QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, "Invalid input data for crop"));
            return;
        }

        vtkPolyData *input = m_params.inputCloud;
        vtkMatrix4x4 *mat = m_params.compositeMatrix;
        double width = m_params.screenWidth;
        double height = m_params.screenHeight;

        vtkSmartPointer<vtkPoints> newPoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> newVerts = vtkSmartPointer<vtkCellArray>::New();
        bool hasResult = false;

        // --- 矩形裁剪逻辑 ---
        // 对应原代码: if (m_currentCropMode == Crop_Rect && m_hasRectSelection)
        if (m_params.mode == 1)
        { // Crop_Rect = 1
            double x_min = m_params.rectX;
            double x_max = m_params.rectX + m_params.rectW;
            double y_min = m_params.rectY;
            double y_max = m_params.rectY + m_params.rectH;

            for (vtkIdType i = 0; i < input->GetNumberOfPoints(); i++)
            {
                double p[3];
                input->GetPoint(i, p);

                // 投影计算
                // 考虑Actor的位置变换
                double worldP[3] = {
                    p[0] + m_params.actorPos[0],
                    p[1] + m_params.actorPos[1],
                    p[2] + m_params.actorPos[2]};
                double view[4] = {worldP[0], worldP[1], worldP[2], 1.0};
                mat->MultiplyPoint(view, view);

                if (view[3] == 0.0)
                    continue;
                double ndcX = view[0] / view[3];
                double ndcY = view[1] / view[3];

                // NDC -> 屏幕坐标
                double screenX = (ndcX + 1.0) * 0.5 * width;
                double screenY = (ndcY + 1.0) * 0.5 * height;

                // 判断是否在矩形内
                bool inside = (screenX >= x_min && screenX <= x_max && screenY >= y_min && screenY <= y_max);

                if ((m_params.cropInside && inside) || (!m_params.cropInside && !inside))
                {
                    vtkIdType id = newPoints->InsertNextPoint(p);
                    newVerts->InsertNextCell(1, &id);
                }
            }
            hasResult = true;
        }
        // --- 多边形裁剪逻辑 ---
        // 对应原代码: else if (m_currentCropMode == Crop_Poly && ...)
        else if (m_params.mode == 2)
        { // Crop_Poly = 2
            const auto &polyVerts = m_params.polyPoints;
            if (polyVerts.size() >= 3)
            {
                for (vtkIdType i = 0; i < input->GetNumberOfPoints(); i++)
                {
                    double p[3];
                    input->GetPoint(i, p);

                    // 投影计算
                    // 考虑Actor的位置变换
                    double worldP[3] = {
                        p[0] + m_params.actorPos[0],
                        p[1] + m_params.actorPos[1],
                        p[2] + m_params.actorPos[2]};
                    double view[4] = {worldP[0], worldP[1], worldP[2], 1.0};
                    mat->MultiplyPoint(view, view);

                    if (view[3] == 0.0)
                        continue;
                    double ndcX = view[0] / view[3];
                    double ndcY = view[1] / view[3];

                    double screenX = (ndcX + 1.0) * 0.5 * width;
                    double screenY = (ndcY + 1.0) * 0.5 * height;

                    // 射线法判断
                    bool inside = false;
                    size_t n = polyVerts.size();
                    for (size_t j = 0, k = n - 1; j < n; k = j++)
                    {
                        if (((polyVerts[j].second > screenY) != (polyVerts[k].second > screenY)) &&
                            (screenX < (polyVerts[k].first - polyVerts[j].first) * (screenY - polyVerts[j].second) / (polyVerts[k].second - polyVerts[j].second) + polyVerts[j].first))
                        {
                            inside = !inside;
                        }
                    }

                    if ((m_params.cropInside && inside) || (!m_params.cropInside && !inside))
                    {
                        vtkIdType id = newPoints->InsertNextPoint(p);
                        newVerts->InsertNextCell(1, &id);
                    }
                }
                hasResult = true;
            }
            else
            {
                QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, "Not enough points for polygon crop"));
                return;
            }
        }

        if (hasResult)
        {
            vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
            output->SetPoints(newPoints);
            output->SetVerts(newVerts);

            // 任务完成，将结果传回 PointDeal
            // 注意：这里使用 QMetaObject::invokeMethod 确保在接收者所在的线程（即主线程）中执行回调
            QMetaObject::invokeMethod(m_receiver, "onTaskFinished", Q_ARG(vtkSmartPointer<vtkPolyData>, output));
        }
        else
        {
            QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, "No crop mode matched or empty result"));
        }
    } catch (const std::exception& e) {
        // 捕获标准异常
        QString errorMsg = QString::fromStdString(std::string("CropTask error: ") + e.what());
        QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, errorMsg));
    } catch (...) {
        // 捕获所有其他异常
        QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, "CropTask: Unknown error occurred"));
    }
}

// ==========================================
// RosToVtkTask 实现
// ==========================================

RosToVtkTask::RosToVtkTask(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, QObject *receiver)
    : m_msg(msg), m_receiver(receiver)
{
    setAutoDelete(true);
}

void RosToVtkTask::run()
{
    try {
        if (!m_msg)
            return;

        // 1. 预分配内存
        size_t numPoints = m_msg->width * m_msg->height;
        if (numPoints == 0)
            return;

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        points->Allocate(numPoints);

        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");
        colors->Allocate(numPoints * 3);

        // 2. 使用迭代器遍历 (参考 方案说明.md 4.4 优化清单)
        // 迭代器能够自动处理字段偏移和步长，比裸指针更安全且符合 ROS2 标准
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*m_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*m_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*m_msg, "z");

        // 检查是否存在 RGB 字段
        bool has_rgb = false;
        for (const auto &field : m_msg->fields)
        {
            if (field.name == "rgb" || field.name == "rgba")
            {
                has_rgb = true;
                break;
            }
        }

        // 3. 遍历点云
        // 注意：只在确认有RGB字段时才创建RGB迭代器，否则会抛出异常
        if (has_rgb)
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*m_msg, "rgb");
            for (size_t i = 0; i < numPoints; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
            {
                float x = *iter_x;
                float y = *iter_y;
                float z = *iter_z;

                // 过滤无效点 (NaN/Inf) 和 极大噪点 (优化清单 4.5)
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    continue;
                if (std::abs(x) > 100.0f || std::abs(y) > 100.0f || std::abs(z) > 100.0f)
                    continue; // 简单阈值过滤

                // 坐标变换：反转Y轴 (Camera Frame -> VTK World Frame)
                points->InsertNextPoint(x, y, z);

                // 解析 RGB
                const uint32_t &rgb_val = *reinterpret_cast<const uint32_t *>(&(*iter_rgb));
                uint8_t r = (rgb_val >> 16) & 0xff;
                uint8_t g = (rgb_val >> 8) & 0xff;
                uint8_t b = (rgb_val) & 0xff;
                colors->InsertNextTuple3(r, g, b);
            }
        }
        else
        {
            // 无RGB字段，使用默认白色
            for (size_t i = 0; i < numPoints; ++i, ++iter_x, ++iter_y, ++iter_z)
            {
                float x = *iter_x;
                float y = *iter_y;
                float z = *iter_z;

                // 过滤无效点 (NaN/Inf) 和 极大噪点
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    continue;
                if (std::abs(x) > 100.0f || std::abs(y) > 100.0f || std::abs(z) > 100.0f)
                    continue;

                // 坐标变换：反转Y轴 (Camera Frame -> VTK World Frame)
                points->InsertNextPoint(x, y, z);

                // 无颜色字段，默认白色
                colors->InsertNextTuple3(255, 255, 255);
            }
        }

        // 4. 构建 vtkPolyData
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->GetPointData()->SetScalars(colors);

        vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
        // 批量构建 Verts 可以优化，但这里使用简单的 InsertNextCell 也可以
        // 这里的点数可能小于 numPoints (因为过滤了无效点)，所以要用 points->GetNumberOfPoints()
        vtkIdType validPoints = points->GetNumberOfPoints();
        verts->Allocate(validPoints); // 预分配
        for (vtkIdType i = 0; i < validPoints; i++)
        {
            verts->InsertNextCell(1, &i);
        }
        polyData->SetVerts(verts);
        // 5. 回调
        QMetaObject::invokeMethod(m_receiver, "onRosTaskFinished", Q_ARG(vtkSmartPointer<vtkPolyData>, polyData));
    } catch (const std::exception& e) {
        // 捕获标准异常
        QString errorMsg = QString::fromStdString(std::string("RosToVtkTask error: ") + e.what());
        QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, errorMsg));
        QMetaObject::invokeMethod(m_receiver, "onRosTaskFinished",
                                  Q_ARG(vtkSmartPointer<vtkPolyData>,
                                        vtkSmartPointer<vtkPolyData>()));
    } catch (...) {
        // 捕获所有其他异常
        QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, "RosToVtkTask: Unknown error occurred"));
        QMetaObject::invokeMethod(m_receiver, "onRosTaskFinished",
                                  Q_ARG(vtkSmartPointer<vtkPolyData>,
                                        vtkSmartPointer<vtkPolyData>()));
    }
}

// ==========================================
// CloudRebuildTask 实现
// ==========================================

CloudRebuildTask::CloudRebuildTask(const QString &sourceName,
                                   vtkSmartPointer<vtkPolyData> inputCloud,
                                   const QString &outputDir,
                                   QObject *receiver)
    : m_sourceName(sourceName),
      m_inputCloud(inputCloud),
      m_outputDir(outputDir),
      m_receiver(receiver)
{
    setAutoDelete(true);
}

void CloudRebuildTask::reportProgress(int progress)
{
    QMetaObject::invokeMethod(m_receiver, "onReconstructionProgress", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName),
                              Q_ARG(int, progress));
}

void CloudRebuildTask::reportLog(const QString &message)
{
    QMetaObject::invokeMethod(m_receiver, "onReconstructionLog", Qt::QueuedConnection,
                              Q_ARG(QString, m_sourceName),
                              Q_ARG(QString, message));
}

void CloudRebuildTask::run()
{
    auto finish = [this](const QString &meshPath, bool ok, const QString &message) {
        QMetaObject::invokeMethod(m_receiver, "onReconstructionFinished", Qt::QueuedConnection,
                                  Q_ARG(QString, m_sourceName),
                                  Q_ARG(QString, meshPath),
                                  Q_ARG(bool, ok),
                                  Q_ARG(QString, message));
    };

    try {
        if (!m_inputCloud || m_inputCloud->GetNumberOfPoints() < 50) {
            finish({}, false, QStringLiteral("输入点云为空或点数过少"));
            return;
        }

        QDir outDir(m_outputDir);
        if (!outDir.exists() && !outDir.mkpath(QStringLiteral("."))) {
            finish({}, false, QStringLiteral("创建重建输出目录失败: %1").arg(m_outputDir));
            return;
        }

        QString safeName = m_sourceName;
        safeName.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_\\-\\u4e00-\\u9fa5]")), QStringLiteral("_"));
        const QString meshPath = outDir.filePath(QStringLiteral("%1_mesh_%2.ply")
            .arg(safeName, QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));

        reportProgress(3);
        reportLog(QStringLiteral("开始准备点云，点数: %1").arg(m_inputCloud->GetNumberOfPoints()));

        pcl::PointCloud<RebuildPointT>::Ptr cloudRaw(new pcl::PointCloud<RebuildPointT>);
        cloudRaw->reserve(static_cast<std::size_t>(m_inputCloud->GetNumberOfPoints()));

        auto *colors = vtkUnsignedCharArray::SafeDownCast(m_inputCloud->GetPointData()->GetScalars());
        for (vtkIdType i = 0; i < m_inputCloud->GetNumberOfPoints(); ++i) {
            double p[3];
            m_inputCloud->GetPoint(i, p);
            RebuildPointT pt;
            pt.x = static_cast<float>(p[0]);
            pt.y = static_cast<float>(p[1]);
            pt.z = static_cast<float>(p[2]);
            if (colors && colors->GetNumberOfTuples() > i) {
                unsigned char rgb[3] = {220, 220, 220};
                colors->GetTypedTuple(i, rgb);
                pt.r = rgb[0];
                pt.g = rgb[1];
                pt.b = rgb[2];
            } else {
                pt.r = 220;
                pt.g = 220;
                pt.b = 220;
            }
            cloudRaw->push_back(pt);
        }
        cloudRaw->width = static_cast<std::uint32_t>(cloudRaw->size());
        cloudRaw->height = 1;
        cloudRaw->is_dense = false;

        reportProgress(12);
        reportLog(QStringLiteral("移除 NaN 点..."));
        pcl::PointCloud<RebuildPointT>::Ptr cloudClean(new pcl::PointCloud<RebuildPointT>);
        std::vector<int> validIndices;
        pcl::removeNaNFromPointCloud(*cloudRaw, *cloudClean, validIndices);
        if (cloudClean->size() < 50) {
            finish({}, false, QStringLiteral("清理后点数过少: %1").arg(cloudClean->size()));
            return;
        }

        pcl::PointCloud<RebuildPointT>::Ptr colorReference = cloudClean;
        pcl::PointCloud<RebuildPointT>::Ptr cloud = cloudClean;

        RebuildPointT minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        reportProgress(22);
        reportLog(QStringLiteral("包围盒 x[%1,%2] y[%3,%4] z[%5,%6]")
            .arg(minPt.x).arg(maxPt.x).arg(minPt.y).arg(maxPt.y).arg(minPt.z).arg(maxPt.z));

        reportLog(QStringLiteral("开始估计法线..."));
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<RebuildPointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<RebuildPointT>::Ptr normalTree(new pcl::search::KdTree<RebuildPointT>);
        ne.setSearchMethod(normalTree);
        ne.setKSearch(20);
        ne.setViewPoint(0.0f, 0.0f, 0.0f);
        ne.compute(*normals);
        if (normals->size() != cloud->size()) {
            finish({}, false, QStringLiteral("法线数量与点云数量不匹配"));
            return;
        }

        reportProgress(42);
        pcl::PointCloud<RebuildPointT>::Ptr cloudValid(new pcl::PointCloud<RebuildPointT>);
        pcl::PointCloud<pcl::Normal>::Ptr normalsValid(new pcl::PointCloud<pcl::Normal>);
        cloudValid->reserve(cloud->size());
        normalsValid->reserve(normals->size());
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const auto &p = cloud->points[i];
            const auto &n = normals->points[i];
            if (isFinitePoint(p) && isFiniteNormal(n)) {
                cloudValid->push_back(p);
                normalsValid->push_back(n);
            }
        }
        cloudValid->width = static_cast<std::uint32_t>(cloudValid->size());
        cloudValid->height = 1;
        cloudValid->is_dense = true;
        normalsValid->width = static_cast<std::uint32_t>(normalsValid->size());
        normalsValid->height = 1;
        normalsValid->is_dense = true;
        if (cloudValid->size() < 50) {
            finish({}, false, QStringLiteral("有效法线点数过少: %1").arg(cloudValid->size()));
            return;
        }

        reportLog(QStringLiteral("有效法线点数: %1").arg(cloudValid->size()));
        pcl::PointCloud<RebuildPointNormalT>::Ptr cloudWithNormals(new pcl::PointCloud<RebuildPointNormalT>);
        pcl::concatenateFields(*cloudValid, *normalsValid, *cloudWithNormals);

        reportProgress(55);
        reportLog(QStringLiteral("开始泊松重建..."));
        pcl::Poisson<RebuildPointNormalT> poisson;
        poisson.setDepth(9);
        poisson.setScale(1.1);
        poisson.setConfidence(false);
        poisson.setOutputPolygons(false);
        poisson.setInputCloud(cloudWithNormals);

        pcl::PolygonMesh mesh;
        poisson.reconstruct(mesh);
        if (mesh.polygons.empty()) {
            finish({}, false, QStringLiteral("泊松重建产生了空网格"));
            return;
        }

        reportProgress(78);
        reportLog(QStringLiteral("泊松重建完成，顶点数: %1，面片数: %2")
            .arg(mesh.cloud.width * mesh.cloud.height)
            .arg(mesh.polygons.size()));

        reportLog(QStringLiteral("开始传输颜色到网格顶点..."));
        pcl::PointCloud<pcl::PointXYZ>::Ptr meshVerticesXyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *meshVerticesXyz);
        pcl::PointCloud<RebuildPointT>::Ptr meshVerticesRgb(new pcl::PointCloud<RebuildPointT>);
        meshVerticesRgb->reserve(meshVerticesXyz->size());

        pcl::KdTreeFLANN<RebuildPointT> colorTree;
        colorTree.setInputCloud(colorReference);
        std::vector<int> nnIndices(1);
        std::vector<float> nnDistances(1);

        for (const auto &v : meshVerticesXyz->points) {
            RebuildPointT query;
            query.x = v.x;
            query.y = v.y;
            query.z = v.z;

            RebuildPointT colored;
            colored.x = v.x;
            colored.y = v.y;
            colored.z = v.z;
            int found = colorTree.nearestKSearch(query, 1, nnIndices, nnDistances);
            if (found > 0) {
                const auto &src = colorReference->points[nnIndices[0]];
                colored.r = src.r;
                colored.g = src.g;
                colored.b = src.b;
            } else {
                colored.r = 200;
                colored.g = 200;
                colored.b = 200;
            }
            meshVerticesRgb->push_back(colored);
        }

        meshVerticesRgb->width = static_cast<std::uint32_t>(meshVerticesRgb->size());
        meshVerticesRgb->height = 1;
        meshVerticesRgb->is_dense = true;
        pcl::toPCLPointCloud2(*meshVerticesRgb, mesh.cloud);

        reportProgress(92);
        if (pcl::io::savePLYFileBinary(meshPath.toStdString(), mesh) != 0) {
            finish({}, false, QStringLiteral("保存网格失败: %1").arg(meshPath));
            return;
        }

        reportProgress(100);
        finish(meshPath, true, QStringLiteral("彩色网格已保存: %1").arg(meshPath));
    } catch (const std::exception &e) {
        finish({}, false, QStringLiteral("重建异常: %1").arg(e.what()));
    } catch (...) {
        finish({}, false, QStringLiteral("重建发生未知异常"));
    }
}

// ==========================================
// PointDeal 实现
// ==========================================

PointDeal::PointDeal(QObject *parent) : QObject(parent)
{
    // 配置私有线程池
    // 设置最大线程数为系统理想线程数，确保充分利用多核性能
    m_threadPool.setMaxThreadCount(QThread::idealThreadCount());
    // 设置线程过期时间（例如 30秒），避免线程长时间空闲占用资源
    m_threadPool.setExpiryTimeout(30000);
}

PointDeal::~PointDeal()
{
    // 析构时安全退出：
    // 1. 清除所有尚未开始的任务
    m_threadPool.clear();
    // 2. 等待正在运行的任务完成（防止回调到已销毁的对象导致崩溃）
    // 注意：如果任务耗时极长，这里可能会阻塞 UI 线程。
    // 在实际生产中，可能需要更复杂的取消机制（如 atomic bool flag 让任务提前退出）。
    // 但对于短时裁剪任务，waitForDone 是最安全的做法。
    m_threadPool.waitForDone();
}

void PointDeal::requestCrop(const CropParams &params)
{
    // 创建并提交任务到私有线程池
    CropTask *task = new CropTask(params, this);
    m_threadPool.start(task);
}

void PointDeal::requestConvertRosToVtk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    if (!msg)
        return;

    if (m_rosConversionInFlight)
    {
        m_latestPendingRosCloud = msg;
        return;
    }

    m_rosConversionInFlight = true;
    startRosConversion(msg);
}

void PointDeal::startRosConversion(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    RosToVtkTask *task = new RosToVtkTask(msg, this);
    m_threadPool.start(task);
}

void PointDeal::requestCloudRebuild(const QString &sourceName,
                                    vtkSmartPointer<vtkPolyData> inputCloud,
                                    const QString &outputDir)
{
    vtkSmartPointer<vtkPolyData> cloudCopy = vtkSmartPointer<vtkPolyData>::New();
    if (inputCloud)
        cloudCopy->DeepCopy(inputCloud);

    CloudRebuildTask *task = new CloudRebuildTask(sourceName, cloudCopy, outputDir, this);
    m_threadPool.start(task);
}

void PointDeal::requestPathPlanning(const QString &sourceName,
                                    vtkSmartPointer<vtkPolyData> surgicalMesh,
                                    vtkSmartPointer<vtkPolyData> referenceMesh,
                                    int sampleCount,
                                    const QString &rodType,
                                    double voxelSize,
                                    double medialPercentile,
                                    int sectionCount,
                                    double sectionHalfWidth,
                                    int smoothPointsNum,
                                    int smoothWindow,
                                    double excludeOpeningDistance,
                                    int excludeOpeningLayers,
                                    double outletRearExtent,
                                    double safetyForwardExtent,
                                    double safeMarginBottom,
                                    double safeMarginTop)
{
    vtkSmartPointer<vtkPolyData> meshCopy = vtkSmartPointer<vtkPolyData>::New();
    if (surgicalMesh)
        meshCopy->DeepCopy(surgicalMesh);
    vtkSmartPointer<vtkPolyData> referenceCopy = vtkSmartPointer<vtkPolyData>::New();
    if (referenceMesh)
        referenceCopy->DeepCopy(referenceMesh);

    PathPlanningTask *task = new PathPlanningTask(sourceName, meshCopy, referenceCopy,
                                                  sampleCount, rodType,
                                                  voxelSize, medialPercentile, sectionCount,
                                                  sectionHalfWidth, smoothPointsNum, smoothWindow,
                                                  excludeOpeningDistance, excludeOpeningLayers,
                                                  outletRearExtent, safetyForwardExtent,
                                                  safeMarginBottom, safeMarginTop, this);
    m_threadPool.start(task);
}

void PointDeal::requestSlicePlanning(const QString &sourceName,
                                     vtkSmartPointer<vtkPolyData> mesh,
                                     vtkSmartPointer<vtkPolyData> straightAxis,
                                    vtkSmartPointer<vtkPolyData> curvedAxis,
                                    double sliceSpacing,
                                    double outletRearExtent,
                                    double safetyForwardExtent,
                                    double safeMarginBottom,
                                     double safeMarginTop,
                                     double planeScaleRatio)
{
    vtkSmartPointer<vtkPolyData> meshCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> straightCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> curvedCopy = vtkSmartPointer<vtkPolyData>::New();
    if (mesh)
        meshCopy->DeepCopy(mesh);
    if (straightAxis)
        straightCopy->DeepCopy(straightAxis);
    if (curvedAxis)
        curvedCopy->DeepCopy(curvedAxis);

    auto *task = new SlicePlanningTask(sourceName, meshCopy, straightCopy, curvedCopy,
                                       sliceSpacing, outletRearExtent, safetyForwardExtent,
                                       safeMarginBottom, safeMarginTop,
                                       planeScaleRatio, this);
    m_threadPool.start(task);
}

void PointDeal::requestSliceContours(const QString &sourceName,
                                     vtkSmartPointer<vtkPolyData> surgicalMesh,
                                     vtkSmartPointer<vtkPolyData> slicePlanes,
                                     vtkSmartPointer<vtkPolyData> firstSlicePlane)
{
    vtkSmartPointer<vtkPolyData> meshCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> planesCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> firstCopy = vtkSmartPointer<vtkPolyData>::New();
    if (surgicalMesh)
        meshCopy->DeepCopy(surgicalMesh);
    if (slicePlanes)
        planesCopy->DeepCopy(slicePlanes);
    if (firstSlicePlane)
        firstCopy->DeepCopy(firstSlicePlane);

    auto *task = new SliceContourTask(sourceName, meshCopy, planesCopy, firstCopy, this);
    m_threadPool.start(task);
}

void PointDeal::requestContourFitting(const QString &sourceName,
                                      vtkSmartPointer<vtkPolyData> contours,
                                      vtkSmartPointer<vtkPolyData> straightAxis,
                                      vtkSmartPointer<vtkPolyData> curvedAxis,
                                      int fitPointCount,
                                      double coverageThreshold,
                                      int smoothWindow,
                                      double angleBinDeg,
                                      bool trimOpenEnds,
                                      int endCheckCount,
                                      double curvaturePeakRatio,
                                      double curvatureRecoverRatio,
                                      int maxTrimCount,
                                      int minKeepPointCount)
{
    vtkSmartPointer<vtkPolyData> contoursCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> straightCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> curvedCopy = vtkSmartPointer<vtkPolyData>::New();
    if (contours)
        contoursCopy->DeepCopy(contours);
    if (straightAxis)
        straightCopy->DeepCopy(straightAxis);
    if (curvedAxis)
        curvedCopy->DeepCopy(curvedAxis);

    auto *task = new ContourFittingTask(sourceName, contoursCopy, straightCopy, curvedCopy,
                                        fitPointCount, coverageThreshold, smoothWindow,
                                        angleBinDeg, trimOpenEnds, endCheckCount,
                                        curvaturePeakRatio, curvatureRecoverRatio,
                                        maxTrimCount, minKeepPointCount, this);
    m_threadPool.start(task);
}

void PointDeal::requestEqualDosePath(const QString &sourceName,
                                     vtkSmartPointer<vtkPolyData> fittedContours,
                                     vtkSmartPointer<vtkPolyData> straightAxis,
                                     vtkSmartPointer<vtkPolyData> curvedAxis,
                                     double nozzleVerticalLength,
                                     double sprayRodRadius,
                                     double safetyClearance,
                                     int closedSamples,
                                     int openSamples,
                                     double connectionRefAngleDeg,
                                     bool useTopOpenEndpoint,
                                     const QString &openEndpointMode,
                                     const QString &manualRegionRanges,
                                     double manualZeroOffsetDeg,
                                     const QString &angleViewDirection,
                                     const QString &angleIncreaseDirection)
{
    vtkSmartPointer<vtkPolyData> fittedCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> straightCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> curvedCopy = vtkSmartPointer<vtkPolyData>::New();
    if (fittedContours)
        fittedCopy->DeepCopy(fittedContours);
    if (straightAxis)
        straightCopy->DeepCopy(straightAxis);
    if (curvedAxis)
        curvedCopy->DeepCopy(curvedAxis);

    auto *task = new EqualDosePathTask(
        sourceName, fittedCopy, straightCopy, curvedCopy,
        nozzleVerticalLength, sprayRodRadius, safetyClearance, closedSamples, openSamples,
        connectionRefAngleDeg, useTopOpenEndpoint, openEndpointMode,
        manualRegionRanges, manualZeroOffsetDeg,
        angleViewDirection, angleIncreaseDirection, this);
    m_threadPool.start(task);
}

void PointDeal::requestContinuousPath(const QString &sourceName,
                                      vtkSmartPointer<vtkPolyData> sprayPath,
                                      vtkSmartPointer<vtkPolyData> straightAxis,
                                      double maxJoint6SweepDeg,
                                      double minPointSpacing,
                                      double maxTransitionDistance,
                                      bool reverseLayerOrder,
                                      bool autoReverseOpenLayers)
{
    vtkSmartPointer<vtkPolyData> pathCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> straightCopy = vtkSmartPointer<vtkPolyData>::New();
    if (sprayPath)
        pathCopy->DeepCopy(sprayPath);
    if (straightAxis)
        straightCopy->DeepCopy(straightAxis);

    auto *task = new ContinuousPathTask(
        sourceName, pathCopy, straightCopy, maxJoint6SweepDeg,
        minPointSpacing, maxTransitionDistance, reverseLayerOrder,
        autoReverseOpenLayers, this);
    m_threadPool.start(task);
}

void PointDeal::requestNozzlePoses(const QString &sourceName,
                                   vtkSmartPointer<vtkPolyData> fittedContours,
                                   vtkSmartPointer<vtkPolyData> rotationStartMarkers,
                                   vtkSmartPointer<vtkPolyData> straightAxis,
                                   vtkSmartPointer<vtkPolyData> curvedAxis,
                                   bool reverseLayerOrder,
                                   double motionTcpToRoundedTipAxial,
                                   double entryTipStandoff)
{
    vtkSmartPointer<vtkPolyData> contoursCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> markersCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> straightCopy = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> curvedCopy = vtkSmartPointer<vtkPolyData>::New();
    if (fittedContours)
        contoursCopy->DeepCopy(fittedContours);
    if (rotationStartMarkers)
        markersCopy->DeepCopy(rotationStartMarkers);
    if (straightAxis)
        straightCopy->DeepCopy(straightAxis);
    if (curvedAxis)
        curvedCopy->DeepCopy(curvedAxis);

    auto *task = new NozzlePoseTask(sourceName, contoursCopy, markersCopy,
                                    straightCopy, curvedCopy, reverseLayerOrder,
                                    motionTcpToRoundedTipAxial,
                                    entryTipStandoff, this);
    m_threadPool.start(task);
}

void PointDeal::onTaskFinished(vtkSmartPointer<vtkPolyData> result)
{
    emit cropFinished(result);
}

void PointDeal::onRosTaskFinished(vtkSmartPointer<vtkPolyData> result)
{
    if (result)
        emit rosCloudFinished(result);

    if (m_latestPendingRosCloud)
    {
        const auto next = std::move(m_latestPendingRosCloud);
        m_latestPendingRosCloud.reset();
        startRosConversion(next);
        return;
    }

    m_rosConversionInFlight = false;
}

void PointDeal::onReconstructionProgress(QString sourceName, int progress)
{
    emit reconstructionProgress(sourceName, progress);
}

void PointDeal::onReconstructionLog(QString sourceName, QString message)
{
    emit reconstructionLog(sourceName, message);
}

void PointDeal::onReconstructionFinished(QString sourceName,
                                         QString meshFilePath,
                                         bool ok,
                                         QString message)
{
    emit reconstructionFinished(sourceName, meshFilePath, ok, message);
}

void PointDeal::onPathPlanningProgress(QString sourceName, int progress)
{
    emit pathPlanningProgress(sourceName, progress);
}

void PointDeal::onPathPlanningLog(QString sourceName, QString message)
{
    emit pathPlanningLog(sourceName, message);
}

void PointDeal::onPathPlanningFinished(QString sourceName,
                                       vtkSmartPointer<vtkPolyData> sampledCloud,
                                       vtkSmartPointer<vtkPolyData> curvedAxis,
                                       vtkSmartPointer<vtkPolyData> straightAxis,
                                       bool ok,
                                       QString message)
{
    emit pathPlanningFinished(sourceName, sampledCloud, curvedAxis, straightAxis, ok, message);
}

void PointDeal::onSlicePlanningProgress(QString sourceName, int progress)
{
    emit slicePlanningProgress(sourceName, progress);
}

void PointDeal::onSlicePlanningLog(QString sourceName, QString message)
{
    emit slicePlanningLog(sourceName, message);
}

void PointDeal::onSlicePlanningFinished(QString sourceName,
                                        vtkSmartPointer<vtkPolyData> slicePlanes,
                                        vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                        vtkSmartPointer<vtkPolyData> boundingBox,
                                        bool ok,
                                        QString message)
{
    emit slicePlanningFinished(sourceName, slicePlanes, firstSlicePlane, boundingBox, ok, message);
}

void PointDeal::onSliceContourProgress(QString sourceName, int progress)
{
    emit sliceContourProgress(sourceName, progress);
}

void PointDeal::onSliceContourLog(QString sourceName, QString message)
{
    emit sliceContourLog(sourceName, message);
}

void PointDeal::onSliceContourFinished(QString sourceName,
                                       vtkSmartPointer<vtkPolyData> contours,
                                       bool ok,
                                       QString message)
{
    emit sliceContourFinished(sourceName, contours, ok, message);
}

void PointDeal::onContourFittingProgress(QString sourceName, int progress)
{
    emit contourFittingProgress(sourceName, progress);
}

void PointDeal::onContourFittingLog(QString sourceName, QString message)
{
    emit contourFittingLog(sourceName, message);
}

void PointDeal::onContourFittingFinished(QString sourceName,
                                         vtkSmartPointer<vtkPolyData> fittedContours,
                                         bool ok,
                                         QString message)
{
    emit contourFittingFinished(sourceName, fittedContours, ok, message);
}

void PointDeal::onEqualDosePathProgress(QString sourceName, int progress)
{
    emit equalDosePathProgress(sourceName, progress);
}

void PointDeal::onEqualDosePathLog(QString sourceName, QString message)
{
    emit equalDosePathLog(sourceName, message);
}

void PointDeal::onEqualDosePathFinished(QString sourceName,
                                        vtkSmartPointer<vtkPolyData> equalDoseSurface,
                                        vtkSmartPointer<vtkPolyData> sprayPath,
                                        vtkSmartPointer<vtkPolyData> pathConnections,
                                        bool ok,
                                        QString message)
{
    emit equalDosePathFinished(sourceName, equalDoseSurface, sprayPath,
                               pathConnections, ok, message);
}

void PointDeal::onContinuousPathProgress(QString sourceName, int progress)
{
    emit continuousPathProgress(sourceName, progress);
}

void PointDeal::onContinuousPathLog(QString sourceName, QString message)
{
    emit continuousPathLog(sourceName, message);
}

void PointDeal::onContinuousPathFinished(QString sourceName,
                                         vtkSmartPointer<vtkPolyData> continuousPath,
                                         vtkSmartPointer<vtkPolyData> transitions,
                                         vtkSmartPointer<vtkPolyData> resetMarkers,
                                         bool ok,
                                         QString message)
{
    emit continuousPathFinished(sourceName, continuousPath, transitions,
                                resetMarkers, ok, message);
}

void PointDeal::onNozzlePoseProgress(QString sourceName, int progress)
{
    emit nozzlePoseProgress(sourceName, progress);
}

void PointDeal::onNozzlePoseLog(QString sourceName, QString message)
{
    emit nozzlePoseLog(sourceName, message);
}

void PointDeal::onNozzlePoseFinished(QString sourceName,
                                     vtkSmartPointer<vtkPolyData> poseSequence,
                                     vtkSmartPointer<vtkPolyData> posePreview,
                                     bool ok,
                                     QString message)
{
    emit nozzlePoseFinished(sourceName, poseSequence, posePreview, ok, message);
}

void PointDeal::onTaskError(QString msg)
{
    emit errorOccurred(msg);
}
