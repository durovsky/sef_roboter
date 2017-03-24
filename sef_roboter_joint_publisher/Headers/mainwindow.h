/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Technical University Kosice, Slovakia
 *  All rights reserved.
 *
 *  Author: Frantisek Durovsky
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "sef_roboter_ros_control/reference_joint.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    enum Mode { SIMULATION, REAL_ROBOT };

    explicit MainWindow(ros::NodeHandle *nh, int mode);
    ~MainWindow();         

private slots:
    void on_button_move_to_home_clicked();
    void on_button_move_to_slider_goal_clicked();
    void on_button_move_to_spinbox_goal_clicked();
    void on_button_reference_joint_1_clicked();
    void on_button_reference_joint_2_clicked();
    void on_button_reference_joint_3_clicked();
    void on_button_reference_joint_4_clicked();
    void on_button_reference_joint_5_clicked();
    void on_button_reference_joint_6_clicked();
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
    void on_spinbox_slider_duration_valueChanged(int arg1);
    void on_spinbox_spinbox_duration_valueChanged(int arg1);


private:
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    Ui::MainWindow *ui;

    ros::NodeHandle *nh;
    ros::Subscriber sub_joint_states;
    ros::Publisher pub_joint_states;
    ros::Publisher pub_joint_trajectory;
    ros::ServiceClient srv_reference_joint_client;

    int mode;
    int timer_id;

    std::vector<double> slider_goals;
    int slider_trajectory_duration;

    std::vector<double> spinbox_goals;
    int spinbox_trajectory_duration;

    static const int NUM_OF_JOINTS = 6;
    static const int HOMING_TIME = 10;

protected:
    void timerEvent(QTimerEvent *timer_event);

};

#endif // MAINWINDOW_H
