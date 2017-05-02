#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define NUM_OF_JOINTS 6

int main(int argc, char **argv)
{
  ros::init (argc, argv, "sef_roboter_real_time_control");
  ros::NodeHandle nh;

  ros::Publisher pub_joint_trajectory = nh.advertise<trajectory_msgs::JointTrajectory > ("/sef_roboter/velocity_trajectory_controller/command", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::TransformListener listener;
  tf::StampedTransform current_device_pose;
  geometry_msgs::Pose goal_pose;

  //Wait for MoveGroup to come up
  ROS_INFO("Sef Roboter Real Time Control.... initial pause!");
  ros::Duration(5).sleep();

  //MoveIt setup
  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
  robot_state::RobotStatePtr current_state_ptr(group.getCurrentState());
  robot_state::RobotStatePtr goal_state_ptr(group.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup(group.getName());

  trajectory_msgs::JointTrajectory arm_command;
  trajectory_msgs::JointTrajectoryPoint desired_configuration;

  desired_configuration.positions.resize(NUM_OF_JOINTS);
  arm_command.joint_names.resize(NUM_OF_JOINTS);

  std::vector<double> goal_joint_values;

  while(ros::ok())
  {
    // Wait for first Tango data
    ROS_INFO("Waiting for transform");
    listener.waitForTransform("/start_of_service", "/current_device_pose", ros::Time(0), ros::Duration(1));
    try  { listener.lookupTransform("/start_of_service", "/current_device_pose", ros::Time(0), current_device_pose); }
    catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

    ROS_INFO_STREAM("New real time goal");

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

    bool found_approach_ik = goal_state_ptr->setFromIK(joint_model_group, goal_pose, 0, 0);
    if(found_approach_ik)
    {
      goal_state_ptr->copyJointGroupPositions("manipulator", goal_joint_values);

      ROS_INFO_STREAM("Joint goals: " << goal_joint_values[0] << " "
                                      << goal_joint_values[1] << " "
                                      << goal_joint_values[2] << " "
                                      << goal_joint_values[3] << " "
                                      << goal_joint_values[4] << " "
                                      << goal_joint_values[5] << " ");

      std::stringstream joint_name;
      for(int i = 0; i < NUM_OF_JOINTS; ++i)
      {
        joint_name.str("");
        joint_name << "joint_" <<  (i + 1);
        desired_configuration.positions[i] = goal_joint_values[i];
        arm_command.joint_names[i] = joint_name.str();
      }

      arm_command.header.stamp = ros::Time::now();
      arm_command.header.frame_id = "base_link";
      arm_command.points.resize(1);
      arm_command.points[0] = desired_configuration;
      arm_command.points[0].time_from_start = ros::Duration(3.0);

      // Publish trajectory
      pub_joint_trajectory.publish(arm_command);
    }
    else
    {
      ROS_WARN("No IK found");
    }

    // Wait
    ros::Duration(0.05).sleep();
  }

  ros::shutdown();
  return EXIT_SUCCESS;
}
