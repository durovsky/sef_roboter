#include <QApplication>
#include "mainwindow.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sef_roboter_joint_publisher");
    ros::NodeHandle nh;

    QApplication a(argc, argv);
    MainWindow w(&nh);
    w.show();

    return a.exec();
}
