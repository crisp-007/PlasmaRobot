#include "point_deal.h"
#include <QDebug>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <QMetaObject>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
    } catch (...) {
        // 捕获所有其他异常
        QMetaObject::invokeMethod(m_receiver, "onTaskError", Q_ARG(QString, "RosToVtkTask: Unknown error occurred"));
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
    // 如果当前有大量积压任务，可以考虑丢弃旧帧（简单策略：不清空，让线程池排队）
    // 或者实现一个单独的单一任务槽来处理最新的点云
    RosToVtkTask *task = new RosToVtkTask(msg, this);
    m_threadPool.start(task);
}

void PointDeal::onTaskFinished(vtkSmartPointer<vtkPolyData> result)
{
    emit cropFinished(result);
}

void PointDeal::onRosTaskFinished(vtkSmartPointer<vtkPolyData> result)
{
    emit rosCloudFinished(result);
}

void PointDeal::onTaskError(QString msg)
{
    emit errorOccurred(msg);
}
