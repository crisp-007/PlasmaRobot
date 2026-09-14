#ifndef NODECLOUDDATA_H
#define NODECLOUDDATA_H

#include <QString>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

// ─────────────────────────────────────────────────────────────
//  NodeCloudData
//  树节点关联的点云数据结构，预加载时填充
// ─────────────────────────────────────────────────────────────
struct NodeCloudData
{
    QString                          filePath;       ///< PLY/PCD 文件路径
    QString                          displayName;    ///< DB树节点显示名
    QString                          motionPathId;   ///< ROS 唯一路径ID，与界面显示名分离
    QString                          meshFilePath;   ///< 重建后的 mesh 文件路径
    vtkSmartPointer<vtkPolyData>     polyData;       ///< 点云几何数据（可直接喂给 vtkMapper）
    vtkSmartPointer<vtkPolyData>     meshPolyData;   ///< 重建后的 mesh 几何数据
    vtkSmartPointer<vtkPolyData>     surgicalPointCloud; ///< 核心术区点云
    vtkSmartPointer<vtkPolyData>     surgicalMesh;       ///< 核心术区 mesh
    vtkSmartPointer<vtkPolyData>     pathSampleCloud;    ///< 路径规划重采样点云
    vtkSmartPointer<vtkPolyData>     curvedAxis;         ///< 弯曲中心线
    vtkSmartPointer<vtkPolyData>     straightAxis;       ///< 直参考轴
    vtkSmartPointer<vtkPolyData>     slicePlanes;        ///< 普通切片平面
    vtkSmartPointer<vtkPolyData>     firstSlicePlane;    ///< 首个切片平面
    vtkSmartPointer<vtkPolyData>     sliceBoundingBox;   ///< 切片定向包围盒
    vtkSmartPointer<vtkPolyData>     sliceContours;      ///< 切片平面与术区 mesh 的交线
    vtkSmartPointer<vtkPolyData>     fittedSliceContours; ///< B 样条拟合后的分层轮廓
    vtkSmartPointer<vtkPolyData>     rotationStartMarkers; ///< 蓝色参考轴 +u 法线与闭合切片轨迹的交点
    vtkSmartPointer<vtkPolyData>     equalDoseSurface;   ///< 分区着色的等剂量带状面
    vtkSmartPointer<vtkPolyData>     sprayPath;          ///< 各切片层喷杆路径
    vtkSmartPointer<vtkPolyData>     sprayPathConnections; ///< 相邻切片层的连接线
    vtkSmartPointer<vtkPolyData>     continuousSprayPath; ///< 受关节6转角约束的分段喷杆路径
    vtkSmartPointer<vtkPolyData>     continuousPathTransitions; ///< 分段路径的层间过渡线
    vtkSmartPointer<vtkPolyData>     joint6ResetMarkers; ///< 分段处的关节6复位标记
    vtkSmartPointer<vtkPolyData>     nozzlePoseSequence; ///< 离线喷嘴六维几何位姿序列
    vtkSmartPointer<vtkPolyData>     nozzlePosePreview;  ///< 稀疏喷嘴方向箭头预览
    std_msgs::msg::Header            sourceHeader; ///< 原始点云坐标系和采集时间
    sensor_msgs::msg::JointState     captureJointState; ///< 点云采集时的六关节状态
    bool                             hasCaptureJointState = false;
    bool                             showCurvedAxis = false; ///< 是否显示弯曲中心线
    bool                             showSliceLayers = true; ///< 是否显示切片平面及相关包围框
    bool                             showFittedContours = true; ///< 是否显示拟合轮廓及旋转起点
    bool                             showEqualDoseSurface = true;
    bool                             showSprayPath = true;
    bool                             showContinuousPath = true;
    bool                             showNozzlePoses = true;
    bool                             fullTrajectoryProcessing = false; ///< 完整喷涂轨迹流水线正在运行
    double                           plannedNozzleLength = 0.0; ///< 侧喷口沿深入轴方向的总长度(m)
    double                           plannedOutletRearExtent = 0.0; ///< 运动 TCP 朝开口方向的喷口占用(m)
    double                           plannedSafetyForwardExtent = 0.0; ///< 喷口 TCP 到圆头末端的轴向距离(m)
    double                           plannedShaftRadius = 0.0; ///< 喷杆外半径(m)
    double                           plannedBottomSafetyMargin = 0.0; ///< 蓝轴底部额外安全距离(m)
    double                           plannedOpeningSafetyMargin = 0.0; ///< 蓝轴开口端额外安全距离(m)
    double                           plannedEntryTipStandoff = 0.030; ///< 圆头在物理开口外的入口停靠距离(m)
    double                           plannedSliceSpacing = 0.0; ///< 规划切片间距(m)
    QString                          plannedToolVariant; ///< 内部末端模型标识，不直接显示给操作者
    double                           plannedToolTcpOffset = 0.0; ///< 法兰到运动 TCP 的长度(m)
    double                           plannedSafetyTipOffset = 0.0; ///< 法兰到物理圆头末端的轴向长度(m)
    int                              plannedToolAxis = -1; ///< 喷杆深入方向对应的工具坐标轴(0/1/2)
    double                           surgicalMeshOpacity = 0.92; ///< 术区 mesh 高亮透明度
    double                           bboxMin[3] = {0.0, 0.0, 0.0};
    double                           bboxMax[3] = {0.0, 0.0, 0.0};
    double                           center[3]  = {0.0, 0.0, 0.0};  ///< 点云质心（用于相机聚焦）
};

#endif // NODECLOUDDATA_H
