#include <QApplication>
#include <QMainWindow>
#include <QTimer>
#include <ros/ros.h>
#include <QTextCodec>
#include <QObject>
#include <QPushButton>


#include "gui/debugpage.h"


int main(int argc, char** argv) {
    // 初始化 ROS
    ros::init(argc, argv, "qt_ros_app");
    ros::NodeHandle nh;

    // 初始化 Qt 应用程序
    QApplication app(argc, argv);

    // 设置 Qt 的编码为 UTF-8
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    QFont font("WenQuanYi Zen Hei");
    QApplication::setFont(font);
    // 创建主窗口
    QMainWindow From;
    
    DebugPage debug_ui;
    debug_ui.setupUi(&From);
    debug_ui.init(); // 初始化 UI

    From.show();
    

    // 创建一个定时器，用于定期调用 ros::spinOnce
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&](){
        // 让 ROS 处理所有的回调
        if (ros::ok()) {
            ros::spinOnce();
            // ros::AsyncSpinner spinner(1); // 使用 1 个线程进行 spin
            // spinner.start();
        } else {
            app.quit();
        }
    });

    // 设置定时器的间隔，通常设置为 10ms 左右
    timer.start(50);

    // 启动 Qt 的主循环
    return app.exec();
}
