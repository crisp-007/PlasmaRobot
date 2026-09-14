#ifndef DBTREE_MODEL_H
#define DBTREE_MODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QString>
#include <QList>
#include <memory>

struct NodeCloudData;   // 前置声明

// ─────────────────────────────────────────────────────────────
//  DbtreeItem  —  树的内部节点
// ─────────────────────────────────────────────────────────────
enum NodeType { RootNode, CategoryNode, LeafNode };

class DbtreeItem
{
public:
    explicit DbtreeItem(const QString &name,
                        DbtreeItem *parent = nullptr,
                        NodeType type = LeafNode);
    ~DbtreeItem();

    DbtreeItem *child(int row) const;
    int         childCount() const;
    int         row() const;             ///< 在父节点中的行号
    DbtreeItem *parentItem() const;
    QString     name() const;
    NodeType    type() const;
    QString     parentName() const;
    bool        isCloudVisibilityNode() const;
    bool        isCaptureCloudNode() const;
    bool        isRebuildResultNode() const;
    bool        isTrajectoryPathNode() const;

    std::shared_ptr<NodeCloudData> cloudData() const { return cloudData_; }
    void setCloudData(std::shared_ptr<NodeCloudData> data) { cloudData_ = data; }

    bool checked() const { return checked_; }
    void setChecked(bool v) { checked_ = v; }

    void appendChild(DbtreeItem *item);
    DbtreeItem *takeChild(int row);

private:
    QString               name_;
    DbtreeItem           *parent_;
    QList<DbtreeItem *>   children_;
    NodeType              type_;
    bool                  checked_ = false; ///< 复选框状态，默认未选中
    std::shared_ptr<NodeCloudData> cloudData_;   ///< 关联的点云数据（仅 LeafNode 持有）
};

// ─────────────────────────────────────────────────────────────
//  DbtreeModel  —  自定义 QAbstractItemModel
// ─────────────────────────────────────────────────────────────
class DbtreeModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    explicit DbtreeModel(QObject *parent = nullptr);
    ~DbtreeModel() override;

    // --- QAbstractItemModel interface ---
    QModelIndex   index(int row, int column,
                        const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex   parent(const QModelIndex &index) const override;
    int           rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int           columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant      data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant      headerData(int section, Qt::Orientation orientation,
                             int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    bool           setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    QString        appendCaptureCloud(std::shared_ptr<NodeCloudData> data);
    QString        appendRebuildMesh(std::shared_ptr<NodeCloudData> data);
    QString        appendTrajectoryPath(const QString &baseName, std::shared_ptr<NodeCloudData> data);
    bool           removeItem(const QModelIndex &index);

private:
    void buildTree();
    DbtreeItem *findTopCategory(const QString &name) const;

    DbtreeItem *root_;
};

#endif // DBTREE_MODEL_H
