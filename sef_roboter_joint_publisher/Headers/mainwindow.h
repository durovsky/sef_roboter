#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle *nh);
    ~MainWindow();

private slots:
    void on_button_move_to_home_clicked();
    void on_button_move_to_slider_goal_clicked();
    void on_button_move_to_spinbox_goal_clicked();
    void on_slider_joint_1_valueChanged(int value);
    void on_slider_joint_2_valueChanged(int value);
    void on_slider_joint_3_valueChanged(int value);
    void on_slider_joint_4_valueChanged(int value);
    void on_slider_joint_5_valueChanged(int value);
    void on_slider_joint_6_valueChanged(int value);
    void on_spinbox_joint_1_valueChanged(double arg1);
    void on_spinbox_joint_2_valueChanged(double arg1);
    void on_spinbox_joint_3_valueChanged(double arg1);
    void on_spinbox_joint_4_valueChanged(double arg1);
    void on_spinbox_joint_5_valueChanged(double arg1);
    void on_spinbox_joint_6_valueChanged(double arg1);



private:
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    Ui::MainWindow *ui;

    ros::NodeHandle *nh;
    ros::Subscriber sub_joint_states;
    ros::Publisher pub_joint_trajectory;

    int timer_id;

    std::vector<double> slider_goals;
    double joint_1_slider_goal;
    double joint_2_slider_goal;
    double joint_3_slider_goal;
    double joint_4_slider_goal;
    double joint_5_slider_goal;
    double joint_6_slider_goal;
    int slider_goal_time;

    std::vector<double> spinbox_goals;
    double joint_1_spinbox_goal;
    double joint_2_spinbox_goal;
    double joint_3_spinbox_goal;
    double joint_4_spinbox_goal;
    double joint_5_spinbox_goal;
    double joint_6_spinbox_goal;
    int spinbox_goal_time;

    static const int NUM_OF_JOINTS = 6;
    static const int HOMING_TIME = 10;

protected:
    void timerEvent(QTimerEvent *timer_event);

};

#endif // MAINWINDOW_H
