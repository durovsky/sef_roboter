#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "sef_roboter_button_plan_execute");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::TransformListener listener;
  tf::StampedTransform current_device_pose;
  geometry_msgs::Pose goal_pose;

  //Wait for MoveGroup to come up
  ROS_INFO("Sef Roboter Button Plan Execute.... initial pause!");
  ros::Duration(5).sleep();

  //MoveIt setup
  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup::Plan current_pose;

  group.setPlannerId("RRTConnectkConfigDefault");
  group.setNumPlanningAttempts(5);
  group.setPlanningTime(3);
  group.setGoalTolerance(0.001);

  while(ros::ok())
  {
    ROS_INFO("Press a ENTER to plan and execute: ");
    std::cin.get();

   // Wait for first Tango data
    listener.waitForTransform("/start_of_service", "/current_device_pose", ros::Time(0), ros::Duration(0.2));
    try  { listener.lookupTransform("/start_of_service", "/current_device_pose", ros::Time(0), current_device_pose); }
    catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

    tf::Vector3 current_position = current_device_pose.getOrigin();
    goal_pose.position.x = -current_position.getY();
    goal_pose.position.y = current_position.getX();
    goal_pose.position.z = current_position.getZ();

    tf::Quaternion current_quaternion = current_device_pose.getRotation();
    tf::Quaternion rotation = tf::createQuaternionFromRPY(0, 1.57, 0);

    // Remove start_of_service model rotation
    current_quaternion = current_quaternion * rotation;

    goal_pose.orientation.x = current_quaternion.getX();
    goal_pose.orientation.y = current_quaternion.getY();
    goal_pose.orientation.z = current_quaternion.getZ();
    goal_pose.orientation.w = current_quaternion.getW();

    group.setPoseTarget(goal_pose);

    if(group.plan(current_pose))
      group.execute(current_pose);
    else
      ROS_INFO("Planning failed!");
  }

  ros::shutdown();
  return EXIT_SUCCESS;
}
