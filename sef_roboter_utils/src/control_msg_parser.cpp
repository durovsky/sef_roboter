#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Float64.h>

#define REAL_ROBOT

ros::Publisher pub_joint_1_position_goal;
ros::Publisher pub_joint_2_position_goal;
ros::Publisher pub_joint_3_position_goal;
ros::Publisher pub_joint_4_position_goal;
ros::Publisher pub_joint_5_position_goal;
ros::Publisher pub_joint_6_position_goal;

ros::Publisher pub_joint_1_velocity_goal;
ros::Publisher pub_joint_2_velocity_goal;
ros::Publisher pub_joint_3_velocity_goal;
ros::Publisher pub_joint_4_velocity_goal;
ros::Publisher pub_joint_5_velocity_goal;
ros::Publisher pub_joint_6_velocity_goal;

ros::Publisher pub_joint_1_position_actual;
ros::Publisher pub_joint_2_position_actual;
ros::Publisher pub_joint_3_position_actual;
ros::Publisher pub_joint_4_position_actual;
ros::Publisher pub_joint_5_position_actual;
ros::Publisher pub_joint_6_position_actual;

ros::Publisher pub_joint_1_velocity_actual;
ros::Publisher pub_joint_2_velocity_actual;
ros::Publisher pub_joint_3_velocity_actual;
ros::Publisher pub_joint_4_velocity_actual;
ros::Publisher pub_joint_5_velocity_actual;
ros::Publisher pub_joint_6_velocity_actual;

void Callback(const control_msgs::JointTrajectoryControllerStateConstPtr &msg)
{
  trajectory_msgs::JointTrajectoryPoint desired = msg->desired;
  trajectory_msgs::JointTrajectoryPoint actual = msg->actual;

  std_msgs::Float64 output_msg;

  //----------------------------------------------

  output_msg.data = desired.positions[0];
  pub_joint_1_position_goal.publish(output_msg);

  output_msg.data = desired.positions[1];
  pub_joint_2_position_goal.publish(output_msg);

  output_msg.data = desired.positions[2];
  pub_joint_3_position_goal.publish(output_msg);

  output_msg.data = desired.positions[3];
  pub_joint_4_position_goal.publish(output_msg);

  output_msg.data = desired.positions[4];
  pub_joint_5_position_goal.publish(output_msg);

  output_msg.data = desired.positions[5];
  pub_joint_6_position_goal.publish(output_msg);

  //------------------------------------------------

  output_msg.data = actual.positions[0];
  pub_joint_1_position_actual.publish(output_msg);

  output_msg.data = actual.positions[1];
  pub_joint_2_position_actual.publish(output_msg);

  output_msg.data = actual.positions[2];
  pub_joint_3_position_actual.publish(output_msg);

  output_msg.data = actual.positions[3];
  pub_joint_4_position_actual.publish(output_msg);

  output_msg.data = actual.positions[4];
  pub_joint_5_position_actual.publish(output_msg);

  output_msg.data = actual.positions[5];
  pub_joint_6_position_actual.publish(output_msg);

  //---------------------------------------------------

  output_msg.data = desired.velocities[0];
  pub_joint_1_velocity_goal.publish(output_msg);

  output_msg.data = desired.velocities[1];
  pub_joint_2_velocity_goal.publish(output_msg);

  output_msg.data = desired.velocities[2];
  pub_joint_3_velocity_goal.publish(output_msg);

  output_msg.data = desired.velocities[3];
  pub_joint_4_velocity_goal.publish(output_msg);

  output_msg.data = desired.velocities[4];
  pub_joint_5_velocity_goal.publish(output_msg);

  output_msg.data = desired.velocities[5];
  pub_joint_6_velocity_goal.publish(output_msg);

  //------------------------------------------------

  #ifdef REAL_ROBOT
    output_msg.data = -actual.velocities[0];
    pub_joint_1_velocity_actual.publish(output_msg);

    output_msg.data = -actual.velocities[1];
    pub_joint_2_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[2];
    pub_joint_3_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[3];
    pub_joint_4_velocity_actual.publish(output_msg);

    output_msg.data = -actual.velocities[4];
    pub_joint_5_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[5];
    pub_joint_6_velocity_actual.publish(output_msg);

  #else
    output_msg.data = actual.velocities[0];
    pub_joint_1_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[1];
    pub_joint_2_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[2];
    pub_joint_3_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[3];
    pub_joint_4_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[4];
    pub_joint_5_velocity_actual.publish(output_msg);

    output_msg.data = actual.velocities[5];
    pub_joint_6_velocity_actual.publish(output_msg);

  #endif

}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "control_msg_parser");
  ros::NodeHandle nh;

  #ifdef REAL_ROBOT
    ros::Subscriber sub = nh.subscribe("/sef_roboter/velocity_trajectory_controller/state", 1, &Callback);
  #else
    ros::Subscriber sub = nh.subscribe("/sef_roboter/joints_controller/state", 1, &Callback);
  #endif

  pub_joint_1_position_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_1/position/goal", 1);
  pub_joint_2_position_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_2/position/goal", 1);
  pub_joint_3_position_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_3/position/goal", 1);
  pub_joint_4_position_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_4/position/goal", 1);
  pub_joint_5_position_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_5/position/goal", 1);
  pub_joint_6_position_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_6/position/goal", 1);

  pub_joint_1_velocity_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_1/velocity/goal", 1);
  pub_joint_2_velocity_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_2/velocity/goal", 1);
  pub_joint_3_velocity_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_3/velocity/goal", 1);
  pub_joint_4_velocity_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_4/velocity/goal", 1);
  pub_joint_5_velocity_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_5/velocity/goal", 1);
  pub_joint_6_velocity_goal = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_6/velocity/goal", 1);

  pub_joint_1_position_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_1/position/actual", 1);
  pub_joint_2_position_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_2/position/actual", 1);
  pub_joint_3_position_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_3/position/actual", 1);
  pub_joint_4_position_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_4/position/actual", 1);
  pub_joint_5_position_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_5/position/actual", 1);
  pub_joint_6_position_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_6/position/actual", 1);

  pub_joint_1_velocity_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_1/velocity/actual", 1);
  pub_joint_2_velocity_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_2/velocity/actual", 1);
  pub_joint_3_velocity_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_3/velocity/actual", 1);
  pub_joint_4_velocity_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_4/velocity/actual", 1);
  pub_joint_5_velocity_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_5/velocity/actual", 1);
  pub_joint_6_velocity_actual = nh.advertise<std_msgs::Float64>("/sef_roboter/joint_6/velocity/actual", 1);

  ros::spin();

  return(EXIT_SUCCESS);
}
