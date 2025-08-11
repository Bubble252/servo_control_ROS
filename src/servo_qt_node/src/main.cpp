#include <QApplication>
#include <ros/ros.h>
#include "servo_qt_node/qt_servo_control.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "qt_servo_control_node");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    QtServoControl window(nh);
    window.setWindowTitle("Qt Servo Controller");
    window.resize(800, 600);
    window.show();

    return app.exec();
}

