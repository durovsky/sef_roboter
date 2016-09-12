#include <QApplication>
#include "mainwindow.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sef_roboter_joint_publisher");
    ros::NodeHandle nh;

    QApplication a(argc, argv);

    int mode = MainWindow::SIMULATION;  //by default

    if ((argc == 2) && (std::string(argv[1]) == "real_robot"))
        mode = MainWindow::REAL_ROBOT;

    MainWindow w(&nh, mode);
    w.show();

    return a.exec();
}
