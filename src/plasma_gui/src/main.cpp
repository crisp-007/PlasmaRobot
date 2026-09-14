#include "mainwindow.h"
#include "gui/panel/left_panel/log/logmanager.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include <QLocale>
#include <QTranslator>
#include <QTime>
#include <random>
#include <chrono>

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication a(argc, argv);

    // 初始化日志系统（必须在业务模块使用日志之前）
    LogManager::instance().init("logs");

    a.setApplicationName("低温等离子术中辅助系统");
    a.setOrganizationName("PlasmaRobot");
    a.setDesktopFileName("plasma_gui");
    a.setWindowIcon(QIcon(":/icons/ui_source/winicon.png"));
    // 初始化随机数种子，使用现代C++方式
    std::srand(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "test1_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }
    MainWindow w;
    w.setWindowIcon(QIcon(":/icons/ui_source/winicon.png"));
    w.show();

    int ret = a.exec();

    // 关闭日志系统（保证日志文件 flush 和关闭）
    LogManager::instance().shutdown();

    return ret;
}
