#ifndef DBTREE_VIEW_H
#define DBTREE_VIEW_H

#include <QWidget>
#include <memory>

QT_BEGIN_NAMESPACE
class QTreeView;
class QLabel;
QT_END_NAMESPACE

class DbtreeModel;
struct NodeCloudData;

// ─────────────────────────────────────────────────────────────
//  DbtreeView
//  树数据模型和业务逻辑管理类
//  在内部创建树控件，外部提供属性标签控件
// ─────────────────────────────────────────────────────────────
class DbtreeView : public QWidget
{
    Q_OBJECT
public:
    explicit DbtreeView(QWidget *parent = nullptr);
    ~DbtreeView() override = default;

    // 设置外部的属性标签，该标签由 mainwindow.ui 提供
    void setPropLabel(QLabel *propLabel);

    // 默认折叠残腔采集节点
    void collapseCaptureNode();
    // 逐次展开残腔采集的子节点（点击一次展开一个），返回当前已展开数量
    int revealNextCaptureFrame();
    QString addCaptureCloud(std::shared_ptr<NodeCloudData> data);
    QString addRebuildMesh(std::shared_ptr<NodeCloudData> data);
    QString addTrajectoryPath(const QString &baseName, std::shared_ptr<NodeCloudData> data);
    void updateCloudNode(std::shared_ptr<NodeCloudData> data);
    void syncCheckedCloudVisibility();
    void setPathPlanningMenuEnabled(bool enabled);

signals:
    // 点击叶子节点时发出，携带预加载的点云数据
    void nodeSelected(std::shared_ptr<NodeCloudData> cloudData);
    void cloudVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void rebuildRequested(std::shared_ptr<NodeCloudData> cloudData);
    void recropRequested(std::shared_ptr<NodeCloudData> cloudData);
    void pathPlanningRequested(std::shared_ptr<NodeCloudData> cloudData);
    void surgicalAreaRequested(std::shared_ptr<NodeCloudData> cloudData);
    void slicePlanningRequested(std::shared_ptr<NodeCloudData> cloudData);
    void sliceContourRequested(std::shared_ptr<NodeCloudData> cloudData);
    void contourFittingRequested(std::shared_ptr<NodeCloudData> cloudData);
    void equalDosePlanningRequested(std::shared_ptr<NodeCloudData> cloudData);
    void continuousPathRequested(std::shared_ptr<NodeCloudData> cloudData);
    void nozzlePoseRequested(std::shared_ptr<NodeCloudData> cloudData);
    void fullTrajectoryRequested(std::shared_ptr<NodeCloudData> cloudData);
    void sliceLayersVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void fittedContoursVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void curvedAxisVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void equalDoseSurfaceVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void sprayPathVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void continuousPathVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void nozzlePoseVisibilityChanged(std::shared_ptr<NodeCloudData> cloudData, bool visible);
    void cloudDeleted(std::shared_ptr<NodeCloudData> cloudData);
    // 叶子节点复选框状态变化，携带节点名和选中状态
    void checkStateChanged(const QString &nodeName, bool checked);

private slots:
    void onNodeClicked(const QModelIndex &index);
    void onItemChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles);
    void onContextMenuRequested(const QPoint &pos);

private:
    void setupUi();
    void applyStyle();

    DbtreeModel *model_;
    QTreeView   *tree_view_;      ///< 内部创建的树控件
    QLabel      *prop_label_;     ///< 外部属性标签指针
    int m_captureRevealCount = 0; ///< 残腔采集子节点已展开数量
    bool m_pathPlanningMenuEnabled = false;
};

#endif // DBTREE_VIEW_H
