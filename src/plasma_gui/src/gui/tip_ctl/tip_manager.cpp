/**
 * @file tip_manager.cpp
 * @brief 提示窗口管理器实现文件
 * 
 * 该文件实现了TipManager类，用于管理多个提示窗口的显示、位置、动画和生命周期。
 * 支持最多同时显示3个提示窗口，新窗口固定在位置3显示，旧窗口向上推移。
 */

#include "tip_manager.h"
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QSequentialAnimationGroup>
#include <QEasingCurve>

/**
 * @brief 构造函数
 * @param parent 父窗口指针，用于确定提示窗口的显示位置
 */
TipManager::TipManager(QWidget *parent)
    : QObject(parent)
    , m_parent(parent)
    , m_autoHideTimer(new QTimer(this))
{
    // 连接定时器信号，用于按顺序自动隐藏最旧的窗口
    connect(m_autoHideTimer, &QTimer::timeout, this, &TipManager::onAutoHideOldest);
}

/**
 * @brief 析构函数
 * 
 * 清理所有活动的提示窗口，确保资源正确释放
 */
TipManager::~TipManager()
{
    // 清理所有活动的提示窗口
    for (TipWidget *tip : m_activeTips) {
        tip->deleteLater();
    }
    m_activeTips.clear();
}

/**
 * @brief 显示新的提示窗口
 * @param text 提示文本内容
 * @param status 提示状态类型（成功、危险、警告）
 * @param duration 显示持续时间（毫秒），默认3000ms
 * 
 * 该方法实现以下逻辑：
 * 1. 新窗口始终显示在固定位置3（最下面）
 * 2. 旧窗口向上推移到位置2和1
 * 3. 超过3个窗口时，最上面的窗口会被移除
 * 4. 所有窗口位置变化都有平滑的动画效果
 */
void TipManager::showTip(const QString &text, TipWidget::TipsStatus status)
{
    // 创建新的提示窗口
    TipWidget *newTip = new TipWidget(m_parent);
    newTip->setParams(text, status, KEEP_DURATION);
    
    // 连接隐藏完成信号，当窗口自动隐藏时触发清理
    connect(newTip, &TipWidget::hideFinished, this, &TipManager::onTipHideFinished);
    
    // 检查是否超出最大窗口数限制（3个），移除队尾的旧窗口
    while (m_activeTips.size() >= MAX_TIPS) {
        // 获取队尾的窗口（最后一个，即最旧的窗口），让它向上淡出
        TipWidget *excessTip = m_activeTips.takeLast();
        // 对超出限制的窗口调用hideTip，让它自然淡出并触发hideFinished信号
        excessTip->hideTip();
        // 注意：这里使用takeLast()从列表中移除，避免while循环无限执行
        // 窗口会在淡出动画完成后通过onTipHideFinished进行最终清理
    }
    
    // 将新窗口插入到列表开头（新窗口将显示在固定位置3）
    m_activeTips.prepend(newTip);
    qDebug("队列元素个数：%d", m_activeTips.size());
    
    // 如果这是第一个窗口，启动统一的自动隐藏定时器
    if (m_activeTips.size() == 1) {
        m_autoHideTimer->start(KEEP_DURATION); // 使用KEEP_DURATION宏
    }
    
    // 更新所有窗口的位置，实现新窗口固定显示，旧窗口向上推移的效果
    for (int i = 0; i < m_activeTips.size(); ++i) {
        TipWidget *tip = m_activeTips[i];
        // 位置计算：新窗口（index=0）显示在位置3，旧窗口向上推移到位置2、1
        // 位置3是最下面的固定位置，位置1是最上面
        // 计算公式：positionIndex = (MAX_TIPS - 1) - i
        // 即：index=0 -> position=2, index=1 -> position=1, index=2 -> position=0
        // int positionIndex = (MAX_TIPS - 1) - i;//3为最新，1为最旧
        int positionIndex = i;
        int targetY = calculateTipY(positionIndex);
        
        if (tip == newTip) {
            // 新窗口：设置初始位置并显示（始终在位置3）
            newTip->setInitialPosition(targetY);
            newTip->showTip();
        } else {
            // 现有窗口：动画移动到新位置（向上移动）
            animateTipToPosition(tip, targetY);
        }
    }
}

/**
 * @brief 移除指定的提示窗口
 * @param tip 要移除的提示窗口指针
 * 
 * 该方法执行完整的窗口清理流程：
 * 1. 从活动列表中移除
 * 2. 强制停止所有动画和定时器
 * 3. 断开所有信号连接
 * 4. 隐藏窗口
 * 5. 延迟删除窗口对象
 */
void TipManager::removeTip(TipWidget *tip)
{
    if (!tip) return;
    
    // 从活动列表中移除
    m_activeTips.removeOne(tip);
    
    // 强制停止所有动画和定时器，防止窗口卡住
    tip->forceStop();
    
    // 断开所有信号连接，避免重复处理和内存泄漏
    tip->disconnect();
    
    // 强制隐藏窗口
    tip->hide();
    
    // 延迟删除窗口对象，确保所有操作完成
    tip->deleteLater();
    

}

/**
 * @brief 处理提示窗口隐藏完成信号
 * 
 * 当提示窗口自动隐藏完成时调用此方法
 * 负责移除窗口并重新排列剩余窗口的位置
 */
void TipManager::onTipHideFinished()
{
    // 获取发送信号的提示窗口
    TipWidget *tip = qobject_cast<TipWidget*>(sender());
    
    if (tip) {
        // 从活动列表中移除窗口（必须在删除前移除，避免访问已删除的指针）
        m_activeTips.removeOne(tip);
        
        // 强制停止所有动画和定时器，防止窗口卡住
        tip->forceStop();
        
        // 断开所有信号连接，避免重复处理和内存泄漏
        tip->disconnect();
        
        // 强制隐藏窗口
        tip->hide();
        
        // 延迟删除窗口对象，确保所有操作完成
        tip->deleteLater();
        
        // 更新剩余窗口的位置，确保正确排列
        updateTipPositions();
        
        // 如果还有其他窗口，继续定时器为下一个最旧的窗口
        if (!m_activeTips.isEmpty() && !m_autoHideTimer->isActive()) {
            m_autoHideTimer->start(AUTO_HIDE_INTERVAL); // 自动隐藏间隔后隐藏下一个最旧的窗口
        }
    }
}

/**
 * @brief 更新所有活动提示窗口的位置
 * 
 * 当有窗口被移除后，重新计算并动画调整剩余窗口的位置
 * 确保窗口按照正确的顺序排列（新窗口在下，旧窗口在上）
 */
void TipManager::updateTipPositions()
{
    // 为所有剩余的窗口创建位置更新动画
    for (int i = 0; i < m_activeTips.size(); ++i) {
        TipWidget *tip = m_activeTips[i];
        
        // 跳过正在进行淡出动画的窗口，避免干扰淡出过程
        if (tip->isHiding()) {
            continue;
        }
        
        // 位置计算：与showTip方法保持一致
        // 确保列表中的索引直接对应位置编号
        // int positionIndex = (MAX_TIPS - 1) - i; // 0->2, 1->1, 2->0
        int positionIndex = i;
        int targetY = calculateTipY(positionIndex);
        animateTipToPosition(tip, targetY);
    }
}

/**
 * @brief 将提示窗口动画移动到指定位置
 * @param tip 要移动的提示窗口指针
 * @param targetY 目标Y坐标
 * 
 * 使用平滑的三次贝塞尔曲线动画，持续300毫秒
 * 动画完成后会自动删除动画对象
 */
void TipManager::animateTipToPosition(TipWidget *tip, int targetY)
{
    // 创建位置动画，使用QPropertyAnimation控制窗口位置
    QPropertyAnimation *moveAnim = new QPropertyAnimation(tip, "pos", this);
    moveAnim->setDuration(300); // 动画持续时间300毫秒
    moveAnim->setEasingCurve(QEasingCurve::OutCubic); // 使用三次贝塞尔缓动曲线
    
    // 设置起始和结束位置（只改变Y坐标，X坐标保持不变）
    QPoint currentPos = tip->pos();
    QPoint targetPos(currentPos.x(), targetY);
    
    moveAnim->setStartValue(currentPos);
    moveAnim->setEndValue(targetPos);
    
    // 动画完成后自动删除
    connect(moveAnim, &QPropertyAnimation::finished, moveAnim, &QPropertyAnimation::deleteLater);
    
    moveAnim->start();
}

/**
 * @brief 计算指定索引位置的Y坐标
 * @param index 位置索引（0=最上面，1=中间，2=最下面）
 * @return 计算得到的Y坐标值
 * 
 * 位置布局说明：
 * - 位置0（index=0）：最上面，Y = parentY + TOP_MARGIN
 * - 位置1（index=1）：中间，Y = parentY + TOP_MARGIN + (TIP_HEIGHT + TIP_SPACING)
 * - 位置2（index=2）：最下面，Y = parentY + TOP_MARGIN + 2*(TIP_HEIGHT + TIP_SPACING)
 */
int TipManager::calculateTipY(int index)
{
    // 如果没有父窗口，使用默认的顶部边距
    if (!m_parent) return TOP_MARGIN;
    
    // 获取父窗口在全局坐标系中的位置
    QPoint parentTopLeft = m_parent->mapToGlobal(QPoint(0, 0));
    // 计算提示窗口的Y坐标：父窗口Y坐标 + 顶部边距 + 索引偏移
    // index 0 是最上面的位置，index 2 是最下面的位置
    return parentTopLeft.y() + TOP_MARGIN + index * (TIP_HEIGHT + TIP_SPACING);
}

/**
 * @brief 自动隐藏最旧的提示窗口
 * 
 * 该方法由统一的定时器调用，确保窗口按照创建顺序（FIFO）依次淡出
 * 最旧的窗口位于列表末尾，最新的窗口位于列表开头
 */
void TipManager::onAutoHideOldest()
{
    // 停止定时器，避免重复触发
    m_autoHideTimer->stop();
    
    // 如果有活动窗口，隐藏最旧的窗口（列表末尾）
    if (!m_activeTips.isEmpty()) {
        TipWidget *oldestTip = m_activeTips.last();
        if (oldestTip && !oldestTip->isHiding()) {
            oldestTip->hideTip();
        }
    }
    
    // 如果还有其他窗口，重新启动定时器为下一个窗口
    if (m_activeTips.size() > 1) {
        m_autoHideTimer->start(AUTO_HIDE_INTERVAL); // 自动隐藏间隔后隐藏下一个最旧的窗口
    }
}