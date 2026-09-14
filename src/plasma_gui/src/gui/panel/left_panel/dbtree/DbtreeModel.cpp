#include "DbtreeModel.h"
#include "NodeCloudData.h"

#include <QStyle>
#include <QApplication>
#include <QDir>
#include <QRegularExpression>
#include <QDebug>
#include <QIcon>
#include <QPixmap>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkNew.h>
#include <limits>

// ─────────────────────────────────────────────────────────────
//  静态辅助：从 PLY/PCD 文件加载到 NodeCloudData
// ─────────────────────────────────────────────────────────────
static std::shared_ptr<NodeCloudData> loadCloudData(const QString &filePath)
{
    auto data = std::make_shared<NodeCloudData>();
    data->filePath = filePath;

    // ---------- 1. 读取点云（兼容 .ply 和 .pcd）----------
    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    // 尝试 RGB 点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool loaded = (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filePath.toStdString(), *cloudRgb) != -1) ||
                  (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath.toStdString(), *cloudRgb) != -1);

    if (loaded) {
        for (const auto &pt : cloudRgb->points) {
            pts->InsertNextPoint(pt.x, pt.y, pt.z);
            colors->InsertNextTuple3(pt.r, pt.g, pt.b);
        }
    } else {
        // 尝试 XYZ 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyz(new pcl::PointCloud<pcl::PointXYZ>);
        bool loadedXyz = (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath.toStdString(), *cloudXyz) != -1) ||
                         (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath.toStdString(), *cloudXyz) != -1);
        if (!loadedXyz) {
            qWarning() << "[DbtreeModel] 无法加载点云文件:" << filePath;
            return nullptr;
        }
        for (const auto &pt : cloudXyz->points) {
            pts->InsertNextPoint(pt.x, pt.y, pt.z);
            colors->InsertNextTuple3(220, 220, 220);
        }
    }

    if (pts->GetNumberOfPoints() == 0) {
        qWarning() << "[DbtreeModel] 空点云:" << filePath;
        return nullptr;
    }

    // ---------- 2. 构建 vtkPolyData ----------
    vtkNew<vtkPolyData> pd;
    pd->SetPoints(pts);
    pd->GetPointData()->SetScalars(colors);
    data->polyData = pd;

    // ---------- 3. 计算包围盒 & 质心 ----------
    double bmin[3] = { std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max() };
    double bmax[3] = { std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::lowest() };

    for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); ++i) {
        double p[3];
        pts->GetPoint(i, p);
        for (int d = 0; d < 3; ++d) {
            if (p[d] < bmin[d]) bmin[d] = p[d];
            if (p[d] > bmax[d]) bmax[d] = p[d];
        }
    }

    for (int d = 0; d < 3; ++d) {
        data->bboxMin[d] = bmin[d];
        data->bboxMax[d] = bmax[d];
        data->center[d]  = (bmin[d] + bmax[d]) * 0.5;
    }

    qDebug().noquote()
        << "[DbtreeModel] 预加载:" << QFileInfo(filePath).fileName()
        << "  点数:" << pts->GetNumberOfPoints()
        << "  中心:" << data->center[0] << data->center[1] << data->center[2];

    return data;
}

// ─────────────────────────────────────────────────────────────
//  静态辅助：节点名 → PLY 文件名
// ─────────────────────────────────────────────────────────────
static QString nodeNameToPlyFile(const QString &nodeName)
{
    static const QString kPcdDir =
        QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/pcd_source/");

    // "关键帧N" → pcd0N.ply
    static QRegularExpression re(QStringLiteral("关键帧(\\d+)"));
    auto match = re.match(nodeName);
    if (!match.hasMatch())
        return {};

    int num = match.captured(1).toInt();
    return kPcdDir + QString("pcd%1.ply").arg(num, 2, 10, QChar('0'));
}

// ─────────────────────────────────────────────────────────────
//  DbtreeItem
// ─────────────────────────────────────────────────────────────
DbtreeItem::DbtreeItem(const QString &name, DbtreeItem *parent, NodeType type)
    : name_(name), parent_(parent), type_(type)
{}

DbtreeItem::~DbtreeItem()
{
    qDeleteAll(children_);
}

void DbtreeItem::appendChild(DbtreeItem *item)
{
    children_.append(item);
}

DbtreeItem *DbtreeItem::takeChild(int row)
{
    if (row < 0 || row >= children_.size())
        return nullptr;
    return children_.takeAt(row);
}

DbtreeItem *DbtreeItem::child(int row) const
{
    return (row >= 0 && row < children_.size()) ? children_.at(row) : nullptr;
}

int DbtreeItem::childCount() const { return children_.size(); }

int DbtreeItem::row() const
{
    if (parent_) {
        return parent_->children_.indexOf(const_cast<DbtreeItem *>(this));
    }
    return 0;
}

DbtreeItem *DbtreeItem::parentItem() const { return parent_; }
QString     DbtreeItem::name()       const { return name_; }
NodeType    DbtreeItem::type()       const { return type_; }

QString DbtreeItem::parentName() const
{
    return parent_ ? parent_->name() : QString();
}

bool DbtreeItem::isCaptureCloudNode() const
{
    return type_ == LeafNode && parentName() == QStringLiteral("残腔采集");
}

bool DbtreeItem::isRebuildResultNode() const
{
    return type_ == LeafNode && parentName() == QStringLiteral("残腔重建");
}

bool DbtreeItem::isTrajectoryPathNode() const
{
    return type_ == LeafNode && parentName() == QStringLiteral("轨迹生成");
}

bool DbtreeItem::isCloudVisibilityNode() const
{
    if (type_ != LeafNode)
        return false;

    const QString parent = parentName();
    return parent == QStringLiteral("残腔采集")
        || parent == QStringLiteral("残腔重建")
        || parent == QStringLiteral("轨迹生成");
}

// ─────────────────────────────────────────────────────────────
//  DbtreeModel
// ─────────────────────────────────────────────────────────────
DbtreeModel::DbtreeModel(QObject *parent)
    : QAbstractItemModel(parent), root_(nullptr)
{
    buildTree();
}

DbtreeModel::~DbtreeModel()
{
    delete root_;
}

void DbtreeModel::buildTree()
{
    root_ = new DbtreeItem("重建框栏", nullptr, RootNode);

    // ── 残腔采集 ──
    DbtreeItem *capture = new DbtreeItem("残腔采集", root_, CategoryNode);
    {
        auto *f1 = new DbtreeItem("关键帧1", capture, LeafNode);
        if (auto d = loadCloudData(nodeNameToPlyFile("关键帧1"))) {
            d->displayName = QStringLiteral("关键帧1");
            f1->setCloudData(d);
        }
        capture->appendChild(f1);

        auto *f2 = new DbtreeItem("关键帧2", capture, LeafNode);
        if (auto d = loadCloudData(nodeNameToPlyFile("关键帧2"))) {
            d->displayName = QStringLiteral("关键帧2");
            f2->setCloudData(d);
        }
        capture->appendChild(f2);

        auto *f3 = new DbtreeItem("关键帧3", capture, LeafNode);
        if (auto d = loadCloudData(nodeNameToPlyFile("关键帧3"))) {
            d->displayName = QStringLiteral("关键帧3");
            f3->setCloudData(d);
        }
        capture->appendChild(f3);
    }
    root_->appendChild(capture);

    // ── 残腔重建 ──
    DbtreeItem *recon = new DbtreeItem("残腔重建", root_, CategoryNode);
    root_->appendChild(recon);

    // ── 轨迹生成 ──
    DbtreeItem *traj = new DbtreeItem("轨迹生成", root_, CategoryNode);
    root_->appendChild(traj);
}

QModelIndex DbtreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    DbtreeItem *parentItem = parent.isValid()
        ? static_cast<DbtreeItem *>(parent.internalPointer())
        : root_;

    DbtreeItem *childItem = parentItem->child(row);
    return childItem ? createIndex(row, column, childItem) : QModelIndex();
}

QModelIndex DbtreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    DbtreeItem *childItem  = static_cast<DbtreeItem *>(index.internalPointer());
    DbtreeItem *parentItem = childItem->parentItem();

    if (parentItem == root_)
        return QModelIndex();

    return createIndex(parentItem->row(), 0, parentItem);
}

int DbtreeModel::rowCount(const QModelIndex &parent) const
{
    DbtreeItem *parentItem = parent.isValid()
        ? static_cast<DbtreeItem *>(parent.internalPointer())
        : root_;
    return parentItem->childCount();
}

int DbtreeModel::columnCount(const QModelIndex & /*parent*/) const
{
    return 1;
}

QVariant DbtreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    auto *item = static_cast<DbtreeItem *>(index.internalPointer());

    if (role == Qt::DisplayRole) {
        return item->name();
    }

    if (role == Qt::DecorationRole) {
        static const QString kIconDir =
            QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/point/");
        static const QString kWizardDir =
            QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/wizard/");

        // 残腔采集 → 自定义图标
        if (item->type() == CategoryNode && item->name() == "残腔采集") {
            static QIcon icon(kIconDir + "cj-node-24.png");
            return icon;
        }
        // 关键帧N → 自定义图标
        if (item->type() == LeafNode && item->name().startsWith("关键帧")) {
            static QIcon icon(kIconDir + "gjz-node-24.png");
            return icon;
        }
        // 残腔重建 → 自定义图标
        if (item->type() == CategoryNode && item->name() == "残腔重建") {
            static QIcon icon(kWizardDir + "grid-24.png");
            return icon;
        }
        // 重建N → 自定义图标
        if (item->type() == LeafNode && item->name().startsWith("重建")) {
            static QIcon icon(kWizardDir + "mesh-24.png");
            return icon;
        }
        if (item->type() == LeafNode && item->parentName() == "轨迹生成") {
            static QIcon icon(kWizardDir + "route-24.png");
            if (!icon.isNull())
                return icon;
        }

        // 其余节点保持系统图标
        QStyle *s = QApplication::style();
        switch (item->type()) {
        case RootNode:
            return s->standardIcon(QStyle::SP_DriveHDIcon);
        case CategoryNode:
            return s->standardIcon(QStyle::SP_DirClosedIcon);
        case LeafNode:
            return s->standardIcon(QStyle::SP_FileIcon);
        }
    }

    // 只有需要控制点云显隐的叶子节点显示复选框
    if (role == Qt::CheckStateRole && item->isCloudVisibilityNode()) {
        return item->checked() ? Qt::Checked : Qt::Unchecked;
    }

    return QVariant();
}

QVariant DbtreeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole && section == 0)
        return QString("重建框栏");
    return QVariant();
}

Qt::ItemFlags DbtreeModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    auto *item = static_cast<DbtreeItem *>(index.internalPointer());
    Qt::ItemFlags f = Qt::ItemIsEnabled | Qt::ItemIsSelectable;

    // 只有需要控制点云显隐的叶子节点显示复选框
    if (item->isCloudVisibilityNode())
        f |= Qt::ItemIsUserCheckable;

    return f;
}

bool DbtreeModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (!index.isValid())
        return false;

    auto *item = static_cast<DbtreeItem *>(index.internalPointer());

    if (role == Qt::CheckStateRole && item->isCloudVisibilityNode()) {
        item->setChecked(value.toInt() == Qt::Checked);
        emit dataChanged(index, index, {role});
        return true;
    }

    return false;
}

DbtreeItem *DbtreeModel::findTopCategory(const QString &name) const
{
    if (!root_)
        return nullptr;

    for (int i = 0; i < root_->childCount(); ++i) {
        DbtreeItem *item = root_->child(i);
        if (item && item->type() == CategoryNode && item->name() == name)
            return item;
    }
    return nullptr;
}

QString DbtreeModel::appendCaptureCloud(std::shared_ptr<NodeCloudData> data)
{
    DbtreeItem *capture = findTopCategory(QStringLiteral("残腔采集"));
    if (!capture || !data || !data->polyData)
        return {};

    const int row = capture->childCount();
    const QString nodeName = QStringLiteral("关键帧%1").arg(row + 1);
    data->displayName = nodeName;
    if (data->filePath.isEmpty())
        data->filePath = nodeName;

    QModelIndex parentIndex = createIndex(capture->row(), 0, capture);
    beginInsertRows(parentIndex, row, row);
    auto *item = new DbtreeItem(nodeName, capture, LeafNode);
    item->setCloudData(data);
    capture->appendChild(item);
    endInsertRows();

    return nodeName;
}

QString DbtreeModel::appendRebuildMesh(std::shared_ptr<NodeCloudData> data)
{
    DbtreeItem *rebuild = findTopCategory(QStringLiteral("残腔重建"));
    if (!rebuild || !data || !data->polyData || data->meshFilePath.isEmpty())
        return {};

    const int row = rebuild->childCount();
    const QString nodeName = QStringLiteral("重建%1").arg(row + 1);
    data->displayName = nodeName;

    QModelIndex parentIndex = createIndex(rebuild->row(), 0, rebuild);
    beginInsertRows(parentIndex, row, row);
    auto *item = new DbtreeItem(nodeName, rebuild, LeafNode);
    item->setCloudData(data);
    rebuild->appendChild(item);
    endInsertRows();

    return nodeName;
}

QString DbtreeModel::appendTrajectoryPath(const QString &baseName, std::shared_ptr<NodeCloudData> data)
{
    DbtreeItem *traj = findTopCategory(QStringLiteral("轨迹生成"));
    if (!traj || !data || !data->polyData || !data->straightAxis)
        return {};

    QString nodeName = QStringLiteral("%1路径").arg(baseName.isEmpty() ? QStringLiteral("重建") : baseName);
    QString uniqueName = nodeName;
    int suffix = 2;
    bool exists = true;
    while (exists) {
        exists = false;
        for (int i = 0; i < traj->childCount(); ++i) {
            if (traj->child(i) && traj->child(i)->name() == uniqueName) {
                exists = true;
                uniqueName = QStringLiteral("%1路径%2").arg(baseName.isEmpty() ? QStringLiteral("重建") : baseName).arg(suffix++);
                break;
            }
        }
    }
    nodeName = uniqueName;
    data->displayName = nodeName;

    QModelIndex parentIndex = createIndex(traj->row(), 0, traj);
    const int row = traj->childCount();
    beginInsertRows(parentIndex, row, row);
    auto *item = new DbtreeItem(nodeName, traj, LeafNode);
    item->setCloudData(data);
    traj->appendChild(item);
    endInsertRows();

    return nodeName;
}

bool DbtreeModel::removeItem(const QModelIndex &index)
{
    if (!index.isValid())
        return false;

    auto *item = static_cast<DbtreeItem *>(index.internalPointer());
    if (!item || item->type() == RootNode)
        return false;

    DbtreeItem *parent = item->parentItem();
    if (!parent)
        return false;

    const int row = item->row();
    QModelIndex parentIndex = parent == root_ ? QModelIndex() : createIndex(parent->row(), 0, parent);
    beginRemoveRows(parentIndex, row, row);
    DbtreeItem *removed = parent->takeChild(row);
    endRemoveRows();
    delete removed;
    return true;
}
