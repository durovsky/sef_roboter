#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle *nh) :
    QMainWindow(0),
    ui(new Ui::MainWindow),
    timer_id(0)
{
    ui->setupUi(this);

    // Subscribe to joint_states
    sub_joint_states = nh->subscribe("/sef_roboter/joint_states", 1, &MainWindow::jointStateCallback, this);

    // Initialize publisher for jointTrajectory
    pub_joint_trajectory = nh->advertise<trajectory_msgs::JointTrajectory > ("/sef_roboter/velocity_trajectory_controller/command", 1);

    // Start timer
    timer_id = startTimer(10);

}

MainWindow::~MainWindow()
{

    if(timer_id)
        killTimer(timer_id);

    delete ui;
}

void MainWindow::timerEvent(QTimerEvent *timer_event)
{
    ros::spinOnce();
}

void MainWindow::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    ROS_INFO("JointStateCallback");
    joint_1_actual_value = msg->position[0];
    QString joint_1_actual_value_str = QString::number(joint_1_actual_value, 'f', 4);
    ui->label_actual_value_joint_1->setText(joint_1_actual_value_str);

    joint_2_actual_value = msg->position[1];
    QString joint_2_actual_value_str = QString::number(joint_2_actual_value, 'f', 4);
    ui->label_actual_value_joint_2->setText(joint_2_actual_value_str);

    joint_3_actual_value = msg->position[2];
    QString joint_3_actual_value_str = QString::number(joint_3_actual_value, 'f', 4);
    ui->label_actual_value_joint_3->setText(joint_3_actual_value_str);

    joint_4_actual_value = msg->position[3];
    QString joint_4_actual_value_str = QString::number(joint_4_actual_value, 'f', 4);
    ui->label_actual_value_joint_4->setText(joint_4_actual_value_str);

    joint_5_actual_value = msg->position[4];
    QString joint_5_actual_value_str = QString::number(joint_5_actual_value, 'f', 4);
    ui->label_actual_value_joint_5->setText(joint_5_actual_value_str);

    joint_6_actual_value = msg->position[5];
    QString joint_6_actual_value_str = QString::number(joint_6_actual_value, 'f', 4);
    ui->label_actual_value_joint_6->setText(joint_6_actual_value_str);
}

void MainWindow::on_button_move_to_home_clicked()
{
    
}

void MainWindow::on_button_move_to_goal_slider_clicked()
{

}

void MainWindow::on_button_move_to_goal_spinbox_clicked()
{

}

void MainWindow::on_slider_joint_1_valueChanged(int value)
{
    joint_1_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_2_valueChanged(int value)
{
    joint_2_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_3_valueChanged(int value)
{
    joint_3_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_4_valueChanged(int value)
{
    joint_4_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_5_valueChanged(int value)
{
    joint_5_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_6_valueChanged(int value)
{
    joint_6_slider_goal = double(value)/10;
}

void MainWindow::on_spinbox_joint_1_valueChanged(double arg1)
{
    joint_1_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_2_valueChanged(double arg1)
{
    joint_2_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_3_valueChanged(double arg1)
{
    joint_3_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_4_valueChanged(double arg1)
{
    joint_4_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_5_valueChanged(double arg1)
{
    joint_5_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_6_valueChanged(double arg1)
{
    joint_6_spinbox_goal = arg1;
}
