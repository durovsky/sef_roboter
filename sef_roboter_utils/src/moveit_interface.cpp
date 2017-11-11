#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

bool new_data = false;
std::vector<double>joint_states(6);

void callback(const sensor_msgs::JointState::ConstPtr msg)
{
  ROS_INFO("Callback");
  joint_states[0] = msg->position[0];
  joint_states[1] = msg->position[1];
  joint_states[2] = msg->position[2];
  joint_states[3] = msg->position[3];
  joint_states[4] = msg->position[4];
  joint_states[5] = msg->position[5];
  new_data = true;
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "sef_roboter_moveit_interface");
  ros::NodeHandle nh;

  ros::Subscriber joint_goal_sub = nh.subscribe("/sef_roboter/joint_states_goal", 1, &callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Wait for MoveGroup to come up
  ROS_INFO("Sef Roboter MoveIt Interface .... initial pause!");
  ros::Duration(5).sleep();

  //MoveIt setup
  moveit::planning_interface::MoveGroupInterface group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan current_pose;

  group.setPlannerId("RRTConnectkConfigDefault");
  group.setNumPlanningAttempts(5);
  group.setPlanningTime(3);
  group.setGoalTolerance(0.001);

  while(ros::ok())
  {
    if(new_data == true)
    {
      // Move simulated robot to goal pose
      new_data = false;
      group.setJointValueTarget(joint_states);
      if(group.plan(current_pose))
        group.execute(current_pose);

      ros::Duration(0.1).sleep();
    }
    else
    {
      ros::Duration(0.1).sleep();
    }
  }

  ros::shutdown();
  return EXIT_SUCCESS;
}
