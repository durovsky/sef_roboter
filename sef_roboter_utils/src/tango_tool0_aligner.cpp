#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tango_tool0_aligner");
  ros::NodeHandle nh;

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("tango_visualization", 1);

  ROS_INFO("Tango tool0 aligner started!");

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  tf::StampedTransform orig_tool0_pose;
  tf::StampedTransform current_device_pose;
  tf::Transform final_transform;

  // Wait for first Tango data
  listener.waitForTransform("/start_of_service", "/device", ros::Time(0), ros::Duration(1000));
  ROS_INFO("Tango connected");

  // Save tool0 pose at that time
  listener.waitForTransform("/base_link", "/tool0", ros::Time(0), ros::Duration(1));
  try  { listener.lookupTransform("/base_link", "/tool0", ros::Time(0), orig_tool0_pose); }
  catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

  tf::Vector3 orig_position = orig_tool0_pose.getOrigin();
  tf::Quaternion orig_rotation = orig_tool0_pose.getRotation();

  ros::Rate rate(100.0);
  while(nh.ok())
  {
    // Calculate TF
    try { listener.lookupTransform("/start_of_service", "/device", ros::Time(0), current_device_pose); }
    catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

    tf::Vector3    current_position = current_device_pose.getOrigin();
    tf::Quaternion current_rotation = current_device_pose.getRotation();

    float final_x = orig_position.getY() + current_position.getX();
    float final_y = -orig_position.getX() + current_position.getY();
    float final_z = orig_position.getZ() + current_position.getZ();

    tf::Vector3    final_position;
    final_position.setX(final_x);
    final_position.setY(final_y);
    final_position.setZ(final_z);

    final_transform.setOrigin(final_position);
    final_transform.setRotation(current_rotation);

    broadcaster.sendTransform(tf::StampedTransform(final_transform, ros::Time::now(), "start_of_service", "current_device_pose"));

    // Marker visualization
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/start_of_service";
    marker.header.stamp = ros::Time::now();

    marker.ns = "tango";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = final_x;
    marker.pose.position.y = final_y;
    marker.pose.position.z = final_z;

    marker.pose.orientation.x = current_rotation.getX();
    marker.pose.orientation.y = current_rotation.getY();
    marker.pose.orientation.z = current_rotation.getZ();
    marker.pose.orientation.w = current_rotation.getW();

    marker.scale.x = 0.08;
    marker.scale.y = 0.16;
    marker.scale.z = 0.01;

    marker.color.r = 0.8f;
    marker.color.g = 0.0f;
    marker.color.b = 0.8f;
    marker.color.a = 0.9;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    ros::Duration(0.01).sleep();
  }

  rate.sleep();
  return 0;
}
