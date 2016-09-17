/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Technical University Kosice, Slovakia
 *  All rights reserved.
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

/* Author: Frantisek Durovsky
   Desc:   ros_control main() entry point for controlling sef_roboter in ROS
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <sef_roboter_ros_control/sef_roboter_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sef_roboter_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<sef_roboter_hw_control::SefRoboterHWInterface> sef_roboter_hw_interface
    (new sef_roboter_hw_control::SefRoboterHWInterface(nh));
  sef_roboter_hw_interface->init();

  // Subscribe to drive_1_input_topic
  ros::Subscriber sub_drive_1_input_topic = nh.subscribe("/drive_1_input_topic", 1, 
          &sef_roboter_hw_control::SefRoboterHWInterface::drive01Callback, sef_roboter_hw_interface);
  
  // Subscribe to drive_2_input_topic
  ros::Subscriber sub_drive_2_input_topic = nh.subscribe("/drive_2_input_topic", 1, 
          &sef_roboter_hw_control::SefRoboterHWInterface::drive02Callback, sef_roboter_hw_interface);
  
  // Subscribe to drive_3_input_topic
  ros::Subscriber sub_drive_3_input_topic = nh.subscribe("/drive_3_input_topic", 1, 
          &sef_roboter_hw_control::SefRoboterHWInterface::drive03Callback, sef_roboter_hw_interface);
  
  // Subscribe to drive_4_input_topic
  ros::Subscriber sub_drive_4_input_topic = nh.subscribe("/drive_4_input_topic", 1, 
          &sef_roboter_hw_control::SefRoboterHWInterface::drive04Callback, sef_roboter_hw_interface);
  
  // Subscribe to drive_5_input_topic
  ros::Subscriber sub_drive_5_input_topic = nh.subscribe("/drive_5_input_topic", 1, 
          &sef_roboter_hw_control::SefRoboterHWInterface::drive05Callback, sef_roboter_hw_interface);
  
  // Subscribe to drive_6_input_topic
  ros::Subscriber sub_drive_6_input_topic = nh.subscribe("/drive_6_input_topic", 1, 
          &sef_roboter_hw_control::SefRoboterHWInterface::drive06Callback, sef_roboter_hw_interface);
  
  // Create Homing service
  ros::ServiceServer homing_service = nh.advertiseService("/homing",
          &sef_roboter_hw_control::SefRoboterHWInterface::homingCallback, sef_roboter_hw_interface);

  //Create Reference Joint service
  ros::ServiceServer reference_joint_service = nh.advertiseService("/reference_joint",
          &sef_roboter_hw_control::SefRoboterHWInterface::referenceJointCallback, sef_roboter_hw_interface);
  
  //Wait until communication starts properly
  ros::Duration(10).sleep();
  
  //Initialize drives
  sef_roboter_hw_interface->initDrives();

  //Safety delay
  ros::Duration(1).sleep();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, sef_roboter_hw_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
