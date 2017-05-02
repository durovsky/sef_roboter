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

  tf::StampedTransform orig_tool0_pose;
  tf::StampedTransform orig_tango_pose;
  tf::StampedTransform current_tool_pose;
  tf::StampedTransform current_tango_pose;

  int index;

  float orig_tool_x, orig_tool_y, orig_tool_z;
  float orig_tango_x, orig_tango_y, orig_tango_z;

  float current_tool_x, current_tool_y, current_tool_z;
  float current_tango_x, current_tango_y, current_tango_z;

  float diff_tool_x, diff_tool_y, diff_tool_z;
  float diff_tango_x, diff_tango_y, diff_tango_z;

  float err_x, err_y, err_z;

  // Wait for first Tango data
  listener.waitForTransform("/start_of_service", "/device", ros::Time(0), ros::Duration(1000));
  ROS_INFO("Tango connected");

  // Save tango init pose
  listener.waitForTransform("/start_of_service", "/device", ros::Time(0), ros::Duration(1));
  try  { listener.lookupTransform("/start_of_service", "/device", ros::Time(0), orig_tango_pose); }
  catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

  tf::Vector3 orig_tango_position = orig_tango_pose.getOrigin();
  orig_tango_x = orig_tango_position.getX();
  orig_tango_y = orig_tango_position.getY();
  orig_tango_z = orig_tango_position.getZ();

  double r1_orig, y1_orig, p1_orig;
  double d1_orig, d2_orig,d3_orig;

  tf::Quaternion orig_tango_orientation = orig_tango_pose.getRotation();
  tf::Matrix3x3(orig_tango_orientation).getEulerYPR(y1_orig, p1_orig, r1_orig);

  d1_orig = y1_orig * 180 / M_PI;
  d2_orig = p1_orig * 180 / M_PI;
  d3_orig = r1_orig * 180 / M_PI;

  ROS_INFO_STREAM("Tango orig: [" << orig_tango_x << " " << orig_tango_y << " " << orig_tango_z << "] ["
                                  << d3_orig << " " << d2_orig << " " << d1_orig << "]");

  // Save tool init pose
  listener.waitForTransform("/base_link", "/tango", ros::Time(0), ros::Duration(1));
  try  { listener.lookupTransform("/base_link", "/tango", ros::Time(0), orig_tool0_pose); }
  catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

  tf::Vector3 orig_tool_position = orig_tool0_pose.getOrigin();
  orig_tool_x = orig_tool_position.getX();
  orig_tool_y = orig_tool_position.getY();
  orig_tool_z = orig_tool_position.getZ();

  ROS_INFO_STREAM("Tango tool: [" << orig_tool_x << " " << orig_tool_y << " " << orig_tool_z << "]");

  ros::Rate rate(100.0);

  index = 1;

  while(nh.ok())
  {
    std::cout << "Press Enter for Measurement: " << index;
    std::cin.ignore();

    // Get actual Tango Pose
    try { listener.lookupTransform("/start_of_service", "/device", ros::Time(0), current_tango_pose); }
    catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

    tf::Vector3 current_tango_position = current_tango_pose.getOrigin();
    current_tango_x = current_tango_position.getY();
    current_tango_y = -current_tango_position.getX();
    current_tango_z = current_tango_position.getZ();

    double r1,y1,p1;
    double d1,d2,d3;
    tf::Quaternion current_tango_orientation = current_tango_pose.getRotation();

    tf::Matrix3x3(current_tango_orientation).getEulerYPR(y1,p1,r1);
    d1 = y1 * 180 / M_PI;
    d2 = p1 * 180 / M_PI;
    d3 = r1 * 180 / M_PI;

    // Get actual Tool Pose
    try { listener.lookupTransform("/base_link", "/tango", ros::Time(0), current_tool_pose); }
    catch (tf::TransformException ex) {ROS_ERROR("%s", ex.what()); }

    tf::Vector3 current_tool_position = current_tool_pose.getOrigin();
    current_tool_x = current_tool_position.getX();
    current_tool_y = current_tool_position.getY();
    current_tool_z = current_tool_position.getZ();

    double r2,y2,p2;
    double d4,d5,d6;
    tf::Quaternion current_tool_orientation = current_tool_pose.getRotation();

    tf::Matrix3x3(current_tool_orientation).getEulerYPR(y2,p2,r2);
    d4 = y2 * 180 / M_PI;
    d5 = p2 * 180 / M_PI;
    d6 = r2 * 180 / M_PI;

    ROS_INFO_STREAM("Data: \t" << std::setw(9) << std::fixed << std::setprecision(6)
                    << current_tool_x << "\t"
                    << current_tool_y << "\t"
                    << current_tool_z << "\t\t"
                    << current_tango_x << "\t"
                    << current_tango_y << "\t"
                    << current_tango_z << "\t\t"
                    << d6 << "\t"
                    << d5 << "\t"
                    << d4 << "\t\t"
                    << d3 << "\t"
                    << d2 << "\t"
                    << d1 << "\t\t");

    index++;
    ros::Duration(0.1).sleep();
  }

  rate.sleep();
  return 0;
}
