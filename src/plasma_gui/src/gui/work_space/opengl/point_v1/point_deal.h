#ifndef POINT_DEAL_H
#define POINT_DEAL_H

#include <QObject>
#include <QThreadPool>
#include <QRunnable>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkMatrix4x4.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cmath>

// 预定义 CropMode 枚举 (需与 MainOpengl 一致)
enum CropMode {
    Crop_None = 0,
    Crop_Rect = 1,
    Crop_Poly = 2,
    Crop_Box = 3
};

struct CropParams {
    vtkPolyData* inputCloud = nullptr;
    vtkMatrix4x4* compositeMatrix = nullptr;
    double screenWidth = 0;
    double screenHeight = 0;
    
    // 裁剪参数
    int mode = 0; // 1=Rect, 2=Poly
    bool cropInside = false;
    
    // 矩形参数
    double rectX = 0, rectY = 0, rectW = 0, rectH = 0;
    
    // 多边形参数
    std::vector<std::pair<double, double>> polyPoints;

    // Actor 位置
    double actorPos[3] = {0.0, 0.0, 0.0};
};

// ==========================================
// 任务类定义
// ==========================================

class CropTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    CropTask(const CropParams& params, QObject* receiver);
    void run() override;

private:
    CropParams m_params;
    QObject* m_receiver;
};

class RosToVtkTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    RosToVtkTask(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, QObject* receiver);
    void run() override;

private:
    sensor_msgs::msg::PointCloud2::ConstSharedPtr m_msg;
    QObject* m_receiver;
};

// ==========================================
// PointDeal 类定义
// ==========================================

class PointDeal : public QObject
{
    Q_OBJECT
public:
    explicit PointDeal(QObject *parent = nullptr);
    ~PointDeal();

    // 外部调用接口
    void requestCrop(const CropParams& params);
    void requestConvertRosToVtk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

signals:
    // 处理完成信号
    void cropFinished(vtkSmartPointer<vtkPolyData> result);
    void rosCloudFinished(vtkSmartPointer<vtkPolyData> result);
    void errorOccurred(QString msg);

private:
    QThreadPool m_threadPool;

    // 允许 CropTask 访问私有成员以触发信号（如果需要，或者通过 QMetaObject::invokeMethod）
    friend class CropTask;
    friend class RosToVtkTask;
    
    // 内部处理任务完成的槽函数（通过 invokeMethod 调用）
    Q_INVOKABLE void onTaskFinished(vtkSmartPointer<vtkPolyData> result);
    Q_INVOKABLE void onRosTaskFinished(vtkSmartPointer<vtkPolyData> result);
    Q_INVOKABLE void onTaskError(QString msg);
};

#endif // POINT_DEAL_H
