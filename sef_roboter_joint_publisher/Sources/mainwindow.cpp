#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle *nh, int mode) :
    QMainWindow(0),
    ui(new Ui::MainWindow),
    timer_id(0),
    slider_trajectory_duration(5),
    spinbox_trajectory_duration(5)
{
    ui->setupUi(this);

    slider_goals.resize(NUM_OF_JOINTS);
    spinbox_goals.resize(NUM_OF_JOINTS);

    // Subscribe to joint_states
    sub_joint_states = nh->subscribe("/sef_roboter/joint_states", 1, &MainWindow::jointStateCallback, this);

    ROS_INFO("Mode %d", mode);

    if(mode == SIMULATION)
    {
        // Initialize publisher for simulation mode
        ROS_INFO("Initializing in simulation mode !");
        pub_joint_trajectory = nh->advertise<trajectory_msgs::JointTrajectory > ("/sef_roboter/joints_controller/command", 1);
        ui->label_mode->setText("<font color='green'>Gazebo Sim</font>");
    }
    else if(mode == REAL_ROBOT)
    {
        // Initialize publisher for real robot communication
        ROS_INFO("Initializing in real robot mode !");
        pub_joint_trajectory = nh->advertise<trajectory_msgs::JointTrajectory > ("/sef_roboter/velocity_trajectory_controller/command", 1);
        ui->label_mode->setText("<font color='red'>REAL ROBOT</font>");
    }

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
    QString value_str;

    value_str = QString::number(msg->position[0], 'f', 4);
    ui->label_actual_value_joint_1->setText(value_str);

    value_str = QString::number(msg->position[1], 'f', 4);
    ui->label_actual_value_joint_2->setText(value_str);

    value_str = QString::number(msg->position[2], 'f', 4);
    ui->label_actual_value_joint_3->setText(value_str);

    value_str = QString::number(msg->position[3], 'f', 4);
    ui->label_actual_value_joint_4->setText(value_str);

    value_str = QString::number(msg->position[4], 'f', 4);
    ui->label_actual_value_joint_5->setText(value_str);

    value_str = QString::number(msg->position[5], 'f', 4);
    ui->label_actual_value_joint_6->setText(value_str);
}

void MainWindow::on_button_move_to_home_clicked()
{
    ROS_INFO("Moving to Home Position");

    trajectory_msgs::JointTrajectory arm_command;
    trajectory_msgs::JointTrajectoryPoint desired_configuration;

    // Resize to 6 joints
    desired_configuration.positions.resize(NUM_OF_JOINTS);
    arm_command.joint_names.resize(NUM_OF_JOINTS);

    std::stringstream joint_name;
    for(int i = 0; i < NUM_OF_JOINTS; ++i)
    {
        joint_name.str("");
        joint_name << "joint_" <<  (i + 1);
        desired_configuration.positions[i] = 0.5;
        arm_command.joint_names[i] = joint_name.str();
    }

    arm_command.header.stamp = ros::Time::now();
    arm_command.header.frame_id = "base_link";
    arm_command.points.resize(1);
    arm_command.points[0] = desired_configuration;
    arm_command.points[0].time_from_start = ros::Duration(HOMING_TIME);

    // Publish homing trajectory
    pub_joint_trajectory.publish(arm_command);
}

void MainWindow::on_button_move_to_slider_goal_clicked()
{
    ROS_INFO("Moving to Goal 1: [%f, %f, %f, %f, %f, %f]", slider_goals[0], slider_goals[1], slider_goals[2],
                                                           slider_goals[3], slider_goals[4], slider_goals[5]);
    trajectory_msgs::JointTrajectory arm_command;
    trajectory_msgs::JointTrajectoryPoint desired_configuration;

    // Resize to 6 joints
    desired_configuration.positions.resize(NUM_OF_JOINTS);
    arm_command.joint_names.resize(NUM_OF_JOINTS);


    std::stringstream joint_name;
    for(int i = 0; i < NUM_OF_JOINTS; ++i)
    {
        joint_name.str("");
        joint_name << "joint_" <<  (i + 1);
        desired_configuration.positions[i] = slider_goals[i];
        arm_command.joint_names[i] = joint_name.str();
    }

    arm_command.header.stamp = ros::Time::now();
    arm_command.header.frame_id = "base_link";
    arm_command.points.resize(1);
    arm_command.points[0] = desired_configuration;
    arm_command.points[0].time_from_start = ros::Duration(slider_trajectory_duration);

    // Publish homing trajectory
    pub_joint_trajectory.publish(arm_command);
}

void MainWindow::on_button_move_to_spinbox_goal_clicked()
{
    ROS_INFO("Moving to Goal 2: [%f, %f, %f, %f, %f, %f]", spinbox_goals[0], spinbox_goals[1], spinbox_goals[2],
                                                           spinbox_goals[3], spinbox_goals[4], spinbox_goals[5]);
    trajectory_msgs::JointTrajectory arm_command;
    trajectory_msgs::JointTrajectoryPoint desired_configuration;

    // Resize to 6 joints
    desired_configuration.positions.resize(NUM_OF_JOINTS);
    arm_command.joint_names.resize(NUM_OF_JOINTS);


    std::stringstream joint_name;
    for(int i = 0; i < NUM_OF_JOINTS; ++i)
    {
        joint_name.str("");
        joint_name << "joint_" <<  (i + 1);
        desired_configuration.positions[i] = spinbox_goals[i];
        arm_command.joint_names[i] = joint_name.str();
    }

    arm_command.header.stamp = ros::Time::now();
    arm_command.header.frame_id = "base_link";
    arm_command.points.resize(1);
    arm_command.points[0] = desired_configuration;
    arm_command.points[0].time_from_start = ros::Duration(spinbox_trajectory_duration);

    // Publish homing trajectory
    pub_joint_trajectory.publish(arm_command);
}

void MainWindow::on_slider_joint_1_valueChanged(int value)
{
    slider_goals[0] = double(value)/100;
    QString value_str = QString::number(slider_goals[0], 'f', 3);
    ui->label_slider_value_joint_1->setText(value_str);

}

void MainWindow::on_slider_joint_2_valueChanged(int value)
{
    slider_goals[1] = double(value)/100;
    QString value_str = QString::number(slider_goals[1], 'f', 3);
    ui->label_slider_value_joint_2->setText(value_str);
}

void MainWindow::on_slider_joint_3_valueChanged(int value)
{
    slider_goals[2] = double(value)/100;
    QString value_str = QString::number(slider_goals[2], 'f', 3);
    ui->label_slider_value_joint_3->setText(value_str);
}

void MainWindow::on_slider_joint_4_valueChanged(int value)
{
    slider_goals[3] = double(value)/100;
    QString value_str = QString::number(slider_goals[3], 'f', 3);
    ui->label_slider_value_joint_4->setText(value_str);
}

void MainWindow::on_slider_joint_5_valueChanged(int value)
{
    slider_goals[4] = double(value)/100;
    QString value_str = QString::number(slider_goals[4], 'f', 3);
    ui->label_slider_value_joint_5->setText(value_str);
}

void MainWindow::on_slider_joint_6_valueChanged(int value)
{
    slider_goals[5] = double(value)/100;
    QString value_str = QString::number(slider_goals[5], 'f', 3);
    ui->label_slider_value_joint_6->setText(value_str);
}

void MainWindow::on_spinbox_joint_1_valueChanged(double arg1)
{
    spinbox_goals[0] = arg1;
}

void MainWindow::on_spinbox_joint_2_valueChanged(double arg1)
{
    spinbox_goals[1] = arg1;
}

void MainWindow::on_spinbox_joint_3_valueChanged(double arg1)
{
    spinbox_goals[2] = arg1;
}

void MainWindow::on_spinbox_joint_4_valueChanged(double arg1)
{
    spinbox_goals[3] = arg1;
}

void MainWindow::on_spinbox_joint_5_valueChanged(double arg1)
{
    spinbox_goals[4] = arg1;
}

void MainWindow::on_spinbox_joint_6_valueChanged(double arg1)
{
    spinbox_goals[5] = arg1;
}

void MainWindow::on_spinbox_slider_duration_valueChanged(int arg1)
{
    slider_trajectory_duration = arg1;
}

void MainWindow::on_spinbox_spinbox_duration_valueChanged(int arg1)
{
    spinbox_trajectory_duration = arg1;
}
