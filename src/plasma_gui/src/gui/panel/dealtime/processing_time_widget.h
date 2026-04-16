/**
 * @file processing_time_widget.h
 * @brief 处理时间显示控件头文件
 * @details 提供一个具有动画效果的计时器控件，支持脉冲动画和缩放效果
 * @author Qt开发团队
 * @date 2024
 */

#ifndef PROCESSING_TIME_WIDGET_H
#define PROCESSING_TIME_WIDGET_H

// Qt核心库
#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QElapsedTimer>

// Qt绘图库
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QConicalGradient>

// Qt动画库
#include <QPropertyAnimation>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <QGraphicsOpacityEffect>

// Qt布局库
#include <QVBoxLayout>
#include <QHBoxLayout>

// Qt工具库
#include <QDateTime>
#include <QFont>

// 标准库
#include <cmath>

/**
 * @class ProcessingTimeWidget
 * @brief 处理时间显示控件类
 * @details 继承自QWidget，提供一个具有动画效果的计时器控件
 *          支持开始/停止/重置计时功能，具有脉冲动画和缩放效果
 */
class ProcessingTimeWidget : public QWidget
{
    Q_OBJECT
    
    // 属性声明，用于配合Qt的自动动画系统
    Q_PROPERTY(qreal pulseRadius READ PulseRadius WRITE SetPulseRadius)    ///< 脉冲半径属性
    Q_PROPERTY(qreal pulseOpacity READ PulseOpacity WRITE SetPulseOpacity)  ///< 脉冲透明度属性
    Q_PROPERTY(qreal scaleRadius READ ScaleRadius WRITE SetScaleRadius)     ///< 缩放半径属性

public:
    /**
     * @brief 构造函数
     * @param parent 父控件指针，默认为nullptr
     */
    explicit ProcessingTimeWidget(QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~ProcessingTimeWidget();

    // === 计时控制接口 ===
    
    /**
     * @brief 开始计时
     * @details 启动计时器并开始动画效果
     */
    void StartTiming();
    
    /**
     * @brief 停止计时
     * @details 暂停计时器但保留当前时间
     */
    void StopTiming();
    
    /**
     * @brief 重置计时
     * @details 将时间重置为0并停止动画
     */
    void ResetTiming();
    
    /**
     * @brief 检查计时器是否正在运行
     * @return true表示正在运行，false表示已停止
     */
    bool IsRunning() const;

    // === 动画属性访问方法 ===
    
    /**
     * @brief 获取脉冲半径
     * @return 当前脉冲半径值
     */
    qreal PulseRadius() const { return pulse_radius; }
    
    /**
     * @brief 设置脉冲半径
     * @param radius 脉冲半径值
     */
    void SetPulseRadius(qreal radius);
    
    /**
     * @brief 获取脉冲透明度
     * @return 当前脉冲透明度值
     */
    qreal PulseOpacity() const { return pulse_opacity; }
    
    /**
     * @brief 设置脉冲透明度
     * @param opacity 脉冲透明度值(0.0-1.0)
     */
    void SetPulseOpacity(qreal opacity);
    
    /**
     * @brief 获取缩放半径
     * @return 当前缩放半径值
     */
    qreal ScaleRadius() const { return scale_radius; }
    
    /**
     * @brief 设置缩放半径
     * @param radius 缩放半径值
     */
    void SetScaleRadius(qreal radius);

protected:
    // === Qt事件处理方法 ===
    
    /**
     * @brief 绘制事件处理
     * @param event 绘制事件指针
     * @details 重写此函数以实现自定义绘制效果
     */
    void paintEvent(QPaintEvent *event) override;
    
    /**
     * @brief 窗口大小改变事件处理
     * @param event 大小改变事件指针
     * @details 重新计算控件中心点和最大半径
     */
    void resizeEvent(QResizeEvent *event) override;
    
    /**
     * @brief 鼠标按下事件处理
     * @param event 鼠标事件指针
     * @details 处理鼠标点击并发射clicked信号
     */
    void mousePressEvent(QMouseEvent *event) override;

signals:
    /**
     * @brief 控件点击信号
     * @details 当用户点击控件时发射此信号
     */
    void clicked();

private slots:
    /**
     * @brief 更新时间显示
     * @details 定时器回调函数，用于更新时间标签显示
     */
    void UpdateTime();
    
    /**
     * @brief 脉冲动画完成回调
     * @details 当脉冲动画完成时调用，用于循环播放动画
     */
    void OnPulseAnimationFinished();
    
    /**
     * @brief 更新缩放环效果
     * @details 实时更新缩放环的位置和透明度
     */
    void UpdateScaleRings();

private:
    // === 私有方法 ===
    
    /**
     * @brief 设置用户界面
     * @details 初始化UI界面和布局
     */
    void SetupUI();
    
    /**
     * @brief 设置动画
     * @details 初始化各种动画效果
     */
    void SetupAnimations();
    
    /**
     * @brief 绘制背景
     * @param painter 绘图对象
     * @details 绘制控件的背景效果
     */
    void DrawBackground(QPainter &painter);
    
    /**
     * @brief 绘制脉冲环
     * @param painter 绘图对象
     * @details 绘制脉冲动画效果
     */
    void DrawPulseRings(QPainter &painter);
    
    /**
     * @brief 绘制缩放图案
     * @param painter 绘图对象
     * @param center 中心点坐标
     * @param radius 半径
     * @param opacity 透明度
     * @details 绘制缩放效果图案
     */
    void DrawScalePattern(QPainter &painter, const QPointF &center, qreal radius, qreal opacity);
    
    /**
     * @brief 格式化时间显示
     * @param milliseconds 毫秒数
     * @return 格式化后的时间字符串
     * @details 将毫秒转换为可读的时间格式
     */
    QString FormatTime(qint64 milliseconds);
    
    // === UI组件 ===
    QLabel *time_label;                     ///< 时间显示标签
    
    // === 计时器成员 ===
    QTimer *update_timer;                   ///< 界面更新定时器
    QElapsedTimer *elapsed_timer;           ///< 计时器
    bool is_running;                        ///< 计时器运行状态
    qint64 total_elapsed;                   ///< 总计时时间(毫秒)
    
    // === 动画相关成员 ===
    QPropertyAnimation *pulse_radius_animation;     ///< 脉冲半径动画
    QPropertyAnimation *pulse_opacity_animation;    ///< 脉冲透明度动画
    QPropertyAnimation *scale_radius_animation;     ///< 缩放半径动画
    QSequentialAnimationGroup *pulse_group;         ///< 脉冲动画组(顺序)
    QParallelAnimationGroup *scale_group;           ///< 缩放动画组(并行)
    
    // === 动画属性值 ===
    qreal pulse_radius;                     ///< 当前脉冲半径
    qreal pulse_opacity;                    ///< 当前脉冲透明度
    qreal scale_radius;                     ///< 当前缩放半径
    
    // === 视觉参数 ===
    QPointF center;                         ///< 控件中心点坐标
    qreal max_radius;                       ///< 最大半径
    
    // === 特效相关数据 ===
    
    /**
     * @struct ScaleRing
     * @brief 缩放环结构体
     * @details 描述单个缩放环的属性
     */
    struct ScaleRing {
        qreal radius;                       ///< 环半径
        qreal opacity;                      ///< 环透明度
        qreal rotation;                     ///< 环旋转角度
        int scale_count;                    ///< 刻度数量
    };
    
    QList<ScaleRing> scale_rings;           ///< 缩放环列表
    QTimer *scale_update_timer;             ///< 特效更新定时器
};

#endif // PROCESSING_TIME_WIDGET_H