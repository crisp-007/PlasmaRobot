#include "DbtreeView.h"
#include "DbtreeModel.h"
#include "NodeCloudData.h"

#include <QTreeView>
#include <QLabel>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QPainter>
#include <QDir>
#include <QPainter>
#include <QDir>
#include <functional>

DbtreeView::DbtreeView(QWidget *parent)
    : QWidget(parent), model_(nullptr), tree_view_(nullptr), prop_label_(nullptr)
{
    setupUi();
    applyStyle();
}

void DbtreeView::setupUi()
{
    // 创建数据模型
    model_ = new DbtreeModel(this);

    // 创建树控件
    tree_view_ = new QTreeView(this);
    tree_view_->setModel(model_);
    tree_view_->setHeaderHidden(false);
    tree_view_->header()->setDefaultSectionSize(200);
    tree_view_->header()->model()->setHeaderData(0, Qt::Horizontal, "重建框栏", Qt::DisplayRole);
    tree_view_->expandAll();
    tree_view_->setAnimated(true);
    tree_view_->setUniformRowHeights(true);
    tree_view_->setRootIsDecorated(true);
    tree_view_->header()->setStretchLastSection(true);
    tree_view_->setContextMenuPolicy(Qt::CustomContextMenu);

    // 将树控件放在布局中
    QVBoxLayout *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(0);
    lay->addWidget(tree_view_);

    connect(tree_view_, &QTreeView::clicked,
            this,       &DbtreeView::onNodeClicked);

    // 复选框状态变化时通知外部
    connect(model_, &QAbstractItemModel::dataChanged,
            this,   &DbtreeView::onItemChanged);
    connect(tree_view_, &QTreeView::customContextMenuRequested,
            this, &DbtreeView::onContextMenuRequested);

    // 创建完成后默认折叠"残腔采集"节点，需要时由 btnCapture 逐步展开
    collapseCaptureNode();
}

void DbtreeView::setPropLabel(QLabel *propLabel)
{
    prop_label_ = propLabel;
    if (prop_label_) {
        // 设置属性标签的对齐方式和自动换行
        prop_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        prop_label_->setWordWrap(true);
    }
}

void DbtreeView::setPathPlanningMenuEnabled(bool enabled)
{
    m_pathPlanningMenuEnabled = enabled;
}

void DbtreeView::applyStyle()
{
    // ── 生成复选框勾选图标（白色勾，透明背景）──
    const QString kIndicatorDir = QDir::tempPath() + QStringLiteral("/plasma_gui_indicator/");
    QDir().mkpath(kIndicatorDir);
    {
        QPixmap pm(16, 16);
        pm.fill(Qt::transparent);
        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QPen(Qt::white, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawLine(QPoint(3, 8), QPoint(6, 11));
        p.drawLine(QPoint(6, 11), QPoint(13, 4));
        p.end();
        pm.save(kIndicatorDir + "check_white.png");
    }

    // 树控件样式
    tree_view_->setStyleSheet(QString(R"(
        QTreeView {
            background-color: #2b2b2b;
            color: #e0e0e0;
            border: 1px solid #555555;
            border-bottom: none;
            border-top-left-radius: 10px;
            border-top-right-radius: 10px;
            border-bottom-left-radius: 0px;
            border-bottom-right-radius: 0px;
            font-size: 13px;
            outline: none;
            show-decoration-selected: 0;
        }
        /* viewport 透明，否则它自己的矩形背景会覆盖 QTreeView 的圆角边框 */
        QTreeView::viewport {
            background-color: transparent;
        }
        QHeaderView {
            background-color: transparent;
            border-top-left-radius: 10px;
            border-top-right-radius: 10px;
            border-bottom: 1px solid #555555;
        }
        QHeaderView::section {
            background-color: #333333;
            color: #aaaaaa;
            border: none;
            padding: 4px 8px;
            font-size: 12px;
        }
        /* 表头第一个 section 跟上圆角 */
        QHeaderView::section:first {
            border-top-left-radius: 10px;
        }
        QHeaderView::section:last {
            border-top-right-radius: 10px;
        }

        QScrollBar:vertical {
            background-color: #2b2b2b;
            width: 6px;
        }
        QScrollBar::handle:vertical {
            background-color: #555555;
            border-radius: 3px;
            min-height: 20px;
        }
        QScrollBar::handle:vertical:hover {
            background-color: #777777;
        }
        QScrollBar::add-line:vertical {
            border: none;
            background: none;
            height: 0px;
        }
        QScrollBar::sub-line:vertical {
            border: none;
            background: none;
            height: 0px;
        }

        QScrollBar:horizontal {
            background-color: #2b2b2b;
            height: 6px;
        }
        QScrollBar:handle:horizontal {
            background-color: #555555;
            border-radius: 3px;
            min-width: 20px;
        }
        QScrollBar:handle:horizontal:hover {
            background-color: #777777;
        }
        QScrollBar:add-line:horizontal {
            border: none;
            background: none;
            width: 0px;
        }
        QScrollBar:sub-line:horizontal {
            border: none;
            background: none;
            width: 0px;
        }

        /* ── 复选框指示器：选中时显示白色勾 ── */
        QTreeView::indicator {
            border: 1px solid #888888;
            background-color: #3a3a3a;
        }
        QTreeView::indicator:hover {
            border-color: #aaaaaa;
        }
        QTreeView::indicator:checked {
            image: url(%1);
            background-color: #3a7bd5;
            border: 1px solid #3a7bd5;
        }
        QTreeView::indicator:disabled {
            border-color: #555555;
            background-color: #2b2b2b;
        }
    )").arg(kIndicatorDir + "check_white.png"));

    // 通过调色板控制选中颜色，不与 show-decoration-selected 冲突
    QPalette pal = tree_view_->palette();
    pal.setColor(QPalette::Highlight, QColor("#3a7bd5"));
    pal.setColor(QPalette::HighlightedText, QColor("#ffffff"));
    tree_view_->setPalette(pal);
}

void DbtreeView::onNodeClicked(const QModelIndex &index)
{
    if (!index.isValid()) return;

    const QString nodeName = model_->data(index, Qt::DisplayRole).toString();

    // 更新右侧属性标签
    if (prop_label_)
        prop_label_->setText(QString("<b>节点：</b> %1").arg(nodeName));

    // 从 model index 取出 DbtreeItem，发射点云数据
    auto *item = static_cast<DbtreeItem *>(index.internalPointer());
    if (item && item->isCloudVisibilityNode()) {
        const bool checked = (model_->data(index, Qt::CheckStateRole).toInt() == Qt::Checked);
        if (!checked)
            return;

        auto data = item->cloudData();
        if (data)
            emit nodeSelected(data);
    }
}

void DbtreeView::onItemChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    // 只关心复选框变化
    if (!roles.contains(Qt::CheckStateRole))
        return;

    auto *item = static_cast<DbtreeItem *>(topLeft.internalPointer());
    if (!item || !item->isCloudVisibilityNode())
        return;

    bool checked = (model_->data(topLeft, Qt::CheckStateRole).toInt() == Qt::Checked);
    if (auto data = item->cloudData())
        emit cloudVisibilityChanged(data, checked);
    emit checkStateChanged(item->name(), checked);
}

void DbtreeView::onContextMenuRequested(const QPoint &pos)
{
    const QModelIndex index = tree_view_->indexAt(pos);
    if (!index.isValid())
        return;

    auto *item = static_cast<DbtreeItem *>(index.internalPointer());
    if (!item || !item->isCloudVisibilityNode())
        return;

    auto data = item->cloudData();
    if (!data)
        return;

    QMenu menu(tree_view_);
    QAction *deleteAction = nullptr;

    if (item->isTrajectoryPathNode()) {
        QAction *fullTrajectoryAction = menu.addAction(QStringLiteral("生成完整喷涂轨迹"));
        fullTrajectoryAction->setEnabled(!data->fullTrajectoryProcessing);

        QMenu *visibilityMenu = menu.addMenu(QStringLiteral("显示内容"));
        auto addVisibilityAction = [visibilityMenu](const QString &text, bool checked) {
            QAction *action = visibilityMenu->addAction(text);
            action->setCheckable(true);
            action->setChecked(checked);
            return action;
        };
        QAction *surfaceVisibilityAction = nullptr;
        QAction *sliceLayersVisibilityAction = nullptr;
        QAction *fittedContoursVisibilityAction = nullptr;
        QAction *sprayPathVisibilityAction = nullptr;
        QAction *continuousPathVisibilityAction = nullptr;
        QAction *nozzlePoseVisibilityAction = nullptr;
        QAction *curvedAction = nullptr;
        if ((data->slicePlanes && data->slicePlanes->GetNumberOfCells() > 0) ||
            (data->firstSlicePlane && data->firstSlicePlane->GetNumberOfCells() > 0)) {
            sliceLayersVisibilityAction = addVisibilityAction(
                QStringLiteral("切片层与原始轮廓"), data->showSliceLayers);
        }
        if (data->fittedSliceContours && data->fittedSliceContours->GetNumberOfLines() > 0) {
            fittedContoursVisibilityAction = addVisibilityAction(
                QStringLiteral("拟合轮廓与旋转起点"), data->showFittedContours);
        }
        if (data->equalDoseSurface && data->equalDoseSurface->GetNumberOfCells() > 0) {
            surfaceVisibilityAction = addVisibilityAction(
                QStringLiteral("喷涂分区面"), data->showEqualDoseSurface);
        }
        if (data->sprayPath && data->sprayPath->GetNumberOfLines() > 0) {
            sprayPathVisibilityAction = addVisibilityAction(
                QStringLiteral("喷杆几何路径"), data->showSprayPath);
        }
        if (data->continuousSprayPath && data->continuousSprayPath->GetNumberOfLines() > 0) {
            continuousPathVisibilityAction = addVisibilityAction(
                QStringLiteral("分段连续路径"), data->showContinuousPath);
        }
        if (data->nozzlePosePreview && data->nozzlePosePreview->GetNumberOfPoints() > 0) {
            nozzlePoseVisibilityAction = addVisibilityAction(
                QStringLiteral("喷嘴位姿"), data->showNozzlePoses);
        }
        if (data->curvedAxis && data->curvedAxis->GetNumberOfPoints() > 0) {
            curvedAction = addVisibilityAction(
                QStringLiteral("弯曲中心线"), data->showCurvedAxis);
        }

        QMenu *advancedMenu = menu.addMenu(QStringLiteral("高级处理"));
        advancedMenu->setEnabled(!data->fullTrajectoryProcessing);
        QAction *sliceAction = advancedMenu->addAction(QStringLiteral("生成切片平面"));
        QAction *contourAction = advancedMenu->addAction(QStringLiteral("生成切片轮廓"));
        QAction *fitContourAction = advancedMenu->addAction(QStringLiteral("拟合切片轮廓"));
        QAction *equalDoseAction = advancedMenu->addAction(QStringLiteral("生成喷杆几何路径"));
        QAction *continuousPathAction = advancedMenu->addAction(QStringLiteral("生成分段连续路径"));
        QAction *nozzlePoseAction = advancedMenu->addAction(QStringLiteral("生成喷嘴位姿"));

        menu.addSeparator();
        deleteAction = menu.addAction(QStringLiteral("删除"));

        QAction *chosen = menu.exec(tree_view_->viewport()->mapToGlobal(pos));
        if (!chosen)
            return;

        if (chosen == fullTrajectoryAction) {
            emit fullTrajectoryRequested(data);
        } else if (chosen == sliceAction) {
            emit slicePlanningRequested(data);
        } else if (chosen == contourAction) {
            emit sliceContourRequested(data);
        } else if (chosen == fitContourAction) {
            emit contourFittingRequested(data);
        } else if (chosen == equalDoseAction) {
            emit equalDosePlanningRequested(data);
        } else if (chosen == continuousPathAction) {
            emit continuousPathRequested(data);
        } else if (chosen == nozzlePoseAction) {
            emit nozzlePoseRequested(data);
        } else if (chosen == sliceLayersVisibilityAction) {
            data->showSliceLayers = chosen->isChecked();
            emit sliceLayersVisibilityChanged(data, data->showSliceLayers);
        } else if (chosen == fittedContoursVisibilityAction) {
            data->showFittedContours = chosen->isChecked();
            emit fittedContoursVisibilityChanged(data, data->showFittedContours);
        } else if (chosen == surfaceVisibilityAction) {
            data->showEqualDoseSurface = chosen->isChecked();
            emit equalDoseSurfaceVisibilityChanged(data, data->showEqualDoseSurface);
        } else if (chosen == sprayPathVisibilityAction) {
            data->showSprayPath = chosen->isChecked();
            emit sprayPathVisibilityChanged(data, data->showSprayPath);
        } else if (chosen == continuousPathVisibilityAction) {
            data->showContinuousPath = chosen->isChecked();
            emit continuousPathVisibilityChanged(data, data->showContinuousPath);
        } else if (chosen == nozzlePoseVisibilityAction) {
            data->showNozzlePoses = chosen->isChecked();
            emit nozzlePoseVisibilityChanged(data, data->showNozzlePoses);
        } else if (chosen == curvedAction) {
            data->showCurvedAxis = chosen->isChecked();
            emit curvedAxisVisibilityChanged(data, data->showCurvedAxis);
        } else if (chosen == deleteAction) {
            emit cloudVisibilityChanged(data, false);
            if (model_->removeItem(index))
                emit cloudDeleted(data);
        }
    } else if (item->isRebuildResultNode()) {
        QAction *pathAction = m_pathPlanningMenuEnabled
            ? menu.addAction(QStringLiteral("路径规划"))
            : nullptr;
        QAction *surgicalAction = menu.addAction(QStringLiteral("术区选择"));
        deleteAction = menu.addAction(QStringLiteral("删除"));

        QAction *chosen = menu.exec(tree_view_->viewport()->mapToGlobal(pos));
        if (!chosen)
            return;

        if (pathAction && chosen == pathAction) {
            emit pathPlanningRequested(data);
        } else if (chosen == surgicalAction) {
            emit surgicalAreaRequested(data);
        } else if (chosen == deleteAction) {
            emit cloudVisibilityChanged(data, false);
            if (model_->removeItem(index))
                emit cloudDeleted(data);
        }
    } else {
        QAction *rebuildAction = menu.addAction(QStringLiteral("重建"));
        QAction *cropAction = menu.addAction(QStringLiteral("裁剪"));
        deleteAction = menu.addAction(QStringLiteral("删除"));

        QAction *chosen = menu.exec(tree_view_->viewport()->mapToGlobal(pos));
        if (!chosen)
            return;

        if (chosen == rebuildAction) {
            emit rebuildRequested(data);
        } else if (chosen == cropAction) {
            emit recropRequested(data);
        } else if (chosen == deleteAction) {
            emit cloudVisibilityChanged(data, false);
            if (model_->removeItem(index))
                emit cloudDeleted(data);
        }
    }
}

void DbtreeView::syncCheckedCloudVisibility()
{
    if (!model_)
        return;

    std::function<void(const QModelIndex &)> visit = [&](const QModelIndex &parent) {
        for (int row = 0; row < model_->rowCount(parent); ++row) {
            const QModelIndex idx = model_->index(row, 0, parent);
            auto *item = static_cast<DbtreeItem *>(idx.internalPointer());
            if (!item)
                continue;

            if (item->isCloudVisibilityNode()) {
                if (auto data = item->cloudData()) {
                    const bool checked = (model_->data(idx, Qt::CheckStateRole).toInt() == Qt::Checked);
                    emit cloudVisibilityChanged(data, checked);
                }
            } else if (item->type() != LeafNode) {
                visit(idx);
            }
        }
    };

    visit(QModelIndex());
}

// ─────────────────────────────────────────────────────────────
//  查找顶层 CategoryNode 的索引（仅在根节点下搜索）
// ─────────────────────────────────────────────────────────────
static QModelIndex findTopCategoryIndex(DbtreeModel *model, const QString &name)
{
    for (int i = 0; i < model->rowCount(QModelIndex()); ++i) {
        QModelIndex idx = model->index(i, 0, QModelIndex());
        if (model->data(idx, Qt::DisplayRole).toString() == name)
            return idx;
    }
    return {};
}

void DbtreeView::collapseCaptureNode()
{
    QModelIndex idx = findTopCategoryIndex(model_, "残腔采集");
    if (idx.isValid())
        tree_view_->collapse(idx);
}

int DbtreeView::revealNextCaptureFrame()
{
    QModelIndex captureIdx = findTopCategoryIndex(model_, "残腔采集");
    if (!captureIdx.isValid())
        return 0;

    // 展开父节点以显示子行
    tree_view_->expand(captureIdx);

    const int childCount = model_->rowCount(captureIdx);
    if (m_captureRevealCount >= childCount)
        return m_captureRevealCount;

    // 每调用一次多显示一个子节点
    m_captureRevealCount++;

    // 只显示前 m_captureRevealCount 个，其余隐藏
    for (int i = 0; i < childCount; ++i)
        tree_view_->setRowHidden(i, captureIdx, i >= m_captureRevealCount);

    return m_captureRevealCount;
}

QString DbtreeView::addCaptureCloud(std::shared_ptr<NodeCloudData> data)
{
    if (!model_)
        return {};

    const QString nodeName = model_->appendCaptureCloud(data);
    QModelIndex captureIdx = findTopCategoryIndex(model_, QStringLiteral("残腔采集"));
    if (captureIdx.isValid()) {
        tree_view_->expand(captureIdx);
        m_captureRevealCount = model_->rowCount(captureIdx);
        for (int i = 0; i < m_captureRevealCount; ++i)
            tree_view_->setRowHidden(i, captureIdx, false);
    }
    return nodeName;
}

QString DbtreeView::addRebuildMesh(std::shared_ptr<NodeCloudData> data)
{
    if (!model_)
        return {};

    const QString nodeName = model_->appendRebuildMesh(data);
    QModelIndex rebuildIdx = findTopCategoryIndex(model_, QStringLiteral("残腔重建"));
    if (rebuildIdx.isValid())
        tree_view_->expand(rebuildIdx);
    return nodeName;
}

QString DbtreeView::addTrajectoryPath(const QString &baseName, std::shared_ptr<NodeCloudData> data)
{
    if (!model_)
        return {};

    const QString nodeName = model_->appendTrajectoryPath(baseName, data);
    QModelIndex trajIdx = findTopCategoryIndex(model_, QStringLiteral("轨迹生成"));
    if (trajIdx.isValid())
        tree_view_->expand(trajIdx);
    return nodeName;
}

void DbtreeView::updateCloudNode(std::shared_ptr<NodeCloudData> data)
{
    if (!data)
        return;

    syncCheckedCloudVisibility();
}
