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

/* Author: Frantisek Durovsky Coleman
   Desc:   ros_control hardware interface for the sef_roboter_sr25 powered by SINAMICS S120 Drives
*/

#include <sef_roboter_ros_control/sef_roboter_hw_interface.h>

namespace sef_roboter_hw_control
{

SefRoboterHWInterface::SefRoboterHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  //Create publisher to write command to drive 1 wrapper output topic
  pub_drive_01_output_topic = nh.advertise<std_msgs::UInt8MultiArray>("/drive_1_output_topic", 1);
  pub_drive_02_output_topic = nh.advertise<std_msgs::UInt8MultiArray>("/drive_2_output_topic", 1);
  pub_drive_03_output_topic = nh.advertise<std_msgs::UInt8MultiArray>("/drive_3_output_topic", 1);
  pub_drive_04_output_topic = nh.advertise<std_msgs::UInt8MultiArray>("/drive_4_output_topic", 1);
  pub_drive_05_output_topic = nh.advertise<std_msgs::UInt8MultiArray>("/drive_5_output_topic", 1);
  pub_drive_06_output_topic = nh.advertise<std_msgs::UInt8MultiArray>("/drive_6_output_topic", 1);
  pub_trajectory = nh.advertise<trajectory_msgs::JointTrajectory>("/sef_roboter/velocity_trajectory_controller/command", 1);

  ROS_INFO_NAMED("sef_roboter_hw_interface", "SefRoboterHWInterface Ready.");
}

SefRoboterHWInterface::~SefRoboterHWInterface()
{
}

void SefRoboterHWInterface::initDrives(void)
{
  //Telegram 4
  unsigned char drive_telegram_4[DRIVE_TELEGRAM_TRANSMIT_SIZE];

  //std_msgs::MultiArray variables
  std_msgs::MultiArrayDimension msg_dim;
  std_msgs::UInt8MultiArray msg;

  //-------------------------------------------
  //Send initialization DRIVE telegram
  // - set all necessary bytes except ON/OFF1 to true
  // - set reference point to true
  //-------------------------------------------

  drive_telegram_4[0] =  0b00000100;   //STW1 high
  drive_telegram_4[1] =  0b10111110;   //STW1 low
  drive_telegram_4[2] =  0b00000000;   //NSOLL_B1 high
  drive_telegram_4[3] =  0b00000000;   //NSOLL_B1 low
  drive_telegram_4[4] =  0b00000000;   //NSOLL_B2_high
  drive_telegram_4[5] =  0b00000010;   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0;  //G1_STW high
  drive_telegram_4[9] =   0;  //G1_STW low
  drive_telegram_4[10] =  0;  //G2_STW high
  drive_telegram_4[11] =  0;  //G2_STW low

  // Drive 01
  msg_dim.label = "Drive_1_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();

  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);

  //Publish the message
  pub_drive_01_output_topic.publish(msg);
  ROS_INFO_NAMED("sef_roboter_hw_interface", "Drive 1 initialization telegram sent!");


  // Drive 02
  msg_dim.label = "Drive_2_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();

  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);

  //Publish the message
  pub_drive_02_output_topic.publish(msg);
  ROS_INFO_NAMED("sef_roboter_hw_interface", "Drive 2 initialization telegram sent!");


  // Drive 03
  msg_dim.label = "Drive_3_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();

  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);

  //Publish the message
  pub_drive_03_output_topic.publish(msg);
  ROS_INFO_NAMED("sef_roboter_hw_interface", "Drive 3 initialization telegram sent!");


  // Drive 04
  msg_dim.label = "Drive_4_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();

  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);

  //Publish the message
  pub_drive_04_output_topic.publish(msg);
  ROS_INFO_NAMED("sef_roboter_hw_interface", "Drive 4 initialization telegram sent!");


  // Drive 05
  msg_dim.label = "Drive_5_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();

  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);

  //Publish the message
  pub_drive_05_output_topic.publish(msg);
  ROS_INFO_NAMED("sef_roboter_hw_interface", "Drive 5 initialization telegram sent!");


  // Drive 06
  msg_dim.label = "Drive_6_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  msg.data.clear();

  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);

  //Publish the message
  pub_drive_06_output_topic.publish(msg);
  ROS_INFO_NAMED("sef_roboter_hw_interface", "Drive 6 initialization telegram sent!");
}

bool SefRoboterHWInterface::homingCallback(sef_roboter_ros_control::homing::Request &req,
                                           sef_roboter_ros_control::homing::Response &res)
{
  //Homing
  trajectory_msgs::JointTrajectory armCommand;
  trajectory_msgs::JointTrajectoryPoint desiredConfiguration;

  desiredConfiguration.positions.resize(NUM_OF_JOINTS);
  armCommand.joint_names.resize(NUM_OF_JOINTS);

  std::stringstream jointName;
  for (int i = 0; i < NUM_OF_JOINTS; ++i) {
    jointName.str("");
    jointName << "joint_" << (i + 1);

    desiredConfiguration.positions[i] = 0;          //Move each joint to zero position
    armCommand.joint_names[i] = jointName.str();
  }

  armCommand.header.stamp = ros::Time::now();
  armCommand.header.frame_id = "base_link";
  armCommand.points.resize(1); // only one endpoint
  armCommand.points[0] = desiredConfiguration;
  armCommand.points[0].time_from_start = ros::Duration(20);

  //Send command
  pub_trajectory.publish(armCommand);

  //Wait for movement execution
  ros::Duration(20).sleep();

  // Check actual joint position values
  if((drive_01_actual_position_rad < 0.01) && (drive_01_actual_position_rad > -0.01) &&
     (drive_02_actual_position_rad < 0.01) && (drive_02_actual_position_rad > -0.01) &&
     (drive_03_actual_position_rad < 0.01) && (drive_03_actual_position_rad > -0.01) &&
     (drive_04_actual_position_rad < 0.01) && (drive_04_actual_position_rad > -0.01) &&
     (drive_05_actual_position_rad < 0.01) && (drive_05_actual_position_rad > -0.01) &&
     (drive_06_actual_position_rad < 0.01) && (drive_06_actual_position_rad > -0.01))
  {
    res.completed = true;
    return true;
  }
  else
  {
    res.completed = false;
    return false;
  }
}

void SefRoboterHWInterface::drive01Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data)
{
  
  int received_byte_array[DRIVE_TELEGRAM_RECEIVE_SIZE];
  int position_raw, velocity_raw;
  double motor_position_rad, motor_velocity_rpm;
    
  for(int i = 0; i < DRIVE_TELEGRAM_RECEIVE_SIZE; i++)
    received_byte_array[i] = callback_data->data[i];
   	     
  //Get actual position as 4 unsigned chars 
  position_raw = (received_byte_array[10] << 24) +
	         (received_byte_array[11] << 16) +
	         (received_byte_array[12] << 8)  +
	          received_byte_array[13];

  //Get actual velocity as 4 unsigned chars
  velocity_raw = (received_byte_array[2] << 24) + 
	         (received_byte_array[3] << 16) +
	         (received_byte_array[4] << 8)  +
	          received_byte_array[5];
	      
  //Motor position in radians 
  motor_position_rad = (double)position_raw / RESOLVER_RESOLUTION;
	
  //Joint Position in radians
  drive_01_actual_position_rad = motor_position_rad * 2 * M_PI / DRIVE_01_RATIO;
  
  //Motor Velocity rpm
  motor_velocity_rpm = (double)velocity_raw * (DRIVE_01_MAX_MOTOR_VELOCITY / 0x40000000);
   
  //Joint Velocity rad/s 
  drive_01_actual_velocity_rad_s = motor_velocity_rpm * 2 * M_PI / 60 / DRIVE_01_RATIO; 
  
  ROS_DEBUG("Drive 01: Position: %10.6f, Velocity: %14.10f", drive_01_actual_position_rad,
                                                             drive_01_actual_velocity_rad_s);
  
}

void SefRoboterHWInterface::drive02Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data)
{
  int received_byte_array[DRIVE_TELEGRAM_RECEIVE_SIZE];
  int position_raw, velocity_raw;
  double motor_position_rad, motor_velocity_rpm;
    
  for(int i = 0; i < DRIVE_TELEGRAM_RECEIVE_SIZE; i++)
    received_byte_array[i] = callback_data->data[i];
   	     
  //Get actual position as 4 unsigned chars 
  position_raw = (received_byte_array[10] << 24) +
	         (received_byte_array[11] << 16) +
	         (received_byte_array[12] << 8)  +
	          received_byte_array[13];

  //Get actual velocity as 4 unsigned chars
  velocity_raw = (received_byte_array[2] << 24) + 
	         (received_byte_array[3] << 16) +
	         (received_byte_array[4] << 8)  +
	          received_byte_array[5];
	      
  //Motor position in radians 
  motor_position_rad = (double)position_raw / RESOLVER_RESOLUTION;
	
  //Joint Position in radians
  drive_02_actual_position_rad = motor_position_rad * 2 * M_PI / DRIVE_02_RATIO;
  
  //Motor Velocity rpm
  motor_velocity_rpm = (double)velocity_raw * (DRIVE_02_MAX_MOTOR_VELOCITY / 0x40000000);
   
  //Joint Velocity rad/s 
  drive_02_actual_velocity_rad_s = motor_velocity_rpm * 2 * M_PI / 60 / DRIVE_02_RATIO; 
    
  ROS_DEBUG("Drive 02: Position: %10.6f, Velocity: %14.10f", drive_02_actual_position_rad,
                                                             drive_02_actual_velocity_rad_s);
}

void SefRoboterHWInterface::drive03Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data)
{
  int received_byte_array[DRIVE_TELEGRAM_RECEIVE_SIZE];
  int position_raw, velocity_raw;
  double motor_position_rad, motor_velocity_rpm;
    
  for(int i = 0; i < DRIVE_TELEGRAM_RECEIVE_SIZE; i++)
    received_byte_array[i] = callback_data->data[i];
   	     
  //Get actual position as 4 unsigned chars 
  position_raw = (received_byte_array[10] << 24) +
	         (received_byte_array[11] << 16) +
	         (received_byte_array[12] << 8)  +
	          received_byte_array[13];

  //Get actual velocity as 4 unsigned chars
  velocity_raw = (received_byte_array[2] << 24) + 
	         (received_byte_array[3] << 16) +
	         (received_byte_array[4] << 8)  +
	          received_byte_array[5];
	      
  //Motor position in radians 
  motor_position_rad = (double)position_raw / RESOLVER_RESOLUTION;
	
  //Joint Position in radians
  drive_03_actual_position_rad = motor_position_rad * 2 * M_PI / DRIVE_03_RATIO;
  
  //Motor Velocity rpm
  motor_velocity_rpm = (double)velocity_raw * (DRIVE_03_MAX_MOTOR_VELOCITY / 0x40000000);
   
  //Joint Velocity rad/s 
  drive_03_actual_velocity_rad_s = motor_velocity_rpm * 2 * M_PI / 60 / DRIVE_03_RATIO; 
  
  ROS_DEBUG("Drive 03: Position: %10.6f, Velocity: %14.10f", drive_03_actual_position_rad,
                                                             drive_03_actual_velocity_rad_s);
}

void SefRoboterHWInterface::drive04Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data)
{
  int received_byte_array[DRIVE_TELEGRAM_RECEIVE_SIZE];
  int position_raw, velocity_raw;
  double motor_position_rad, motor_velocity_rpm;
    
  for(int i = 0; i < DRIVE_TELEGRAM_RECEIVE_SIZE; i++)
    received_byte_array[i] = callback_data->data[i];
   	     
  //Get actual position as 4 unsigned chars 
  position_raw = (received_byte_array[10] << 24) +
	         (received_byte_array[11] << 16) +
	         (received_byte_array[12] << 8)  +
	          received_byte_array[13];

  //Get actual velocity as 4 unsigned chars
  velocity_raw = (received_byte_array[2] << 24) + 
	         (received_byte_array[3] << 16) +
	         (received_byte_array[4] << 8)  +
	          received_byte_array[5];
	      
  //Motor position in radians 
  motor_position_rad = (double)position_raw / RESOLVER_RESOLUTION;
	
  //Joint Position in radians
  drive_04_actual_position_rad = motor_position_rad * 2 * M_PI / DRIVE_04_RATIO ;
  
  //Motor Velocity rpm
  motor_velocity_rpm = (double)velocity_raw * (DRIVE_04_MAX_MOTOR_VELOCITY / 0x40000000);
   
  //Joint Velocity rad/s 
  drive_04_actual_velocity_rad_s = motor_velocity_rpm * 2 * M_PI / 60 / DRIVE_04_RATIO; 
      
  ROS_DEBUG("Drive 04: Position: %10.6f, Velocity: %14.10f", drive_04_actual_position_rad,
                                                             drive_04_actual_velocity_rad_s);
}

void SefRoboterHWInterface::drive05Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data)
{
  int received_byte_array[DRIVE_TELEGRAM_RECEIVE_SIZE];
  int position_raw, velocity_raw;
  double motor_position_rad, motor_velocity_rpm;
    
  for(int i = 0; i < DRIVE_TELEGRAM_RECEIVE_SIZE; i++)
    received_byte_array[i] = callback_data->data[i];
   	     
  //Get actual position as 4 unsigned chars 
  position_raw = (received_byte_array[10] << 24) +
	         (received_byte_array[11] << 16) +
	         (received_byte_array[12] << 8)  +
	          received_byte_array[13];

  //Get actual velocity as 4 unsigned chars
  velocity_raw = (received_byte_array[2] << 24) + 
	         (received_byte_array[3] << 16) +
	         (received_byte_array[4] << 8)  +
	          received_byte_array[5];
	      
  //Motor position in radians 
  motor_position_rad = (double)position_raw / RESOLVER_RESOLUTION;
	
  //Joint Position in radians
  drive_05_actual_position_rad = motor_position_rad * 2 * M_PI / DRIVE_05_RATIO;
  
  //Motor Velocity rpm
  motor_velocity_rpm = (double)velocity_raw * (DRIVE_05_MAX_MOTOR_VELOCITY / 0x40000000);
   
  //Joint Velocity rad/s 
  drive_05_actual_velocity_rad_s = motor_velocity_rpm * 2 * M_PI / 60 / DRIVE_05_RATIO; 
    
  ROS_DEBUG("Drive 05: Position: %10.6f, Velocity: %14.10f", drive_05_actual_position_rad,
                                                             drive_05_actual_velocity_rad_s);
}

void SefRoboterHWInterface::drive06Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data)
{
  int received_byte_array[DRIVE_TELEGRAM_RECEIVE_SIZE];
  int position_raw, velocity_raw;
  double motor_position_rad, motor_velocity_rpm;
    
  for(int i = 0; i < DRIVE_TELEGRAM_RECEIVE_SIZE; i++)
    received_byte_array[i] = callback_data->data[i];
   	     
  //Get actual position as 4 unsigned chars 
  position_raw = (received_byte_array[10] << 24) +
	         (received_byte_array[11] << 16) +
	         (received_byte_array[12] << 8)  +
	          received_byte_array[13];

  //Get actual velocity as 4 unsigned chars
  velocity_raw = (received_byte_array[2] << 24) + 
	         (received_byte_array[3] << 16) +
	         (received_byte_array[4] << 8)  +
	          received_byte_array[5];
	      
  //Motor position in radians 
  motor_position_rad = (double)position_raw / RESOLVER_RESOLUTION;
	
  //Joint Position in radians
  drive_06_actual_position_rad = motor_position_rad * 2 * M_PI / DRIVE_06_RATIO;
  
  //Motor Velocity rpm
  motor_velocity_rpm = (double)velocity_raw * (DRIVE_06_MAX_MOTOR_VELOCITY / 0x40000000);
   
  //Joint Velocity rad/s 
  drive_06_actual_velocity_rad_s = motor_velocity_rpm * 2 * M_PI / 60 / DRIVE_06_RATIO; 
   
  ROS_DEBUG("Drive 06: Position: %10.6f, Velocity: %14.10f", drive_06_actual_position_rad,
                                                             drive_06_actual_velocity_rad_s);
}


void SefRoboterHWInterface::read(ros::Duration &elapsed_time)
{
  //Read current values of Drive 01
  joint_position_[0] = -drive_01_actual_position_rad;       //! Changing feedback direction here (CW -> CCW)
  joint_velocity_[0] = drive_01_actual_velocity_rad_s;

  //Read current values of Drive 02
  joint_position_[1] = -drive_02_actual_position_rad;       //! Changing feedback direction here (CW -> CCW)
  joint_velocity_[1] = drive_02_actual_velocity_rad_s;
  
  //Read current values of Drive 03
  joint_position_[2] = drive_03_actual_position_rad;
  joint_velocity_[2] = drive_03_actual_velocity_rad_s;
  
  //Read current values of Drive 04
  joint_position_[3] = drive_04_actual_position_rad;
  joint_velocity_[3] = drive_04_actual_velocity_rad_s;
  
  //Read current values of Drive 05
  joint_position_[4] = -drive_05_actual_position_rad;       //! Changing feedback direction here (CW -> CCW)
  joint_velocity_[4] = drive_05_actual_velocity_rad_s;
  
  //Read current values of Drive 06
  joint_position_[5] = drive_06_actual_position_rad;
  joint_velocity_[5] = drive_06_actual_velocity_rad_s;
  
}

void SefRoboterHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  //Telegram PZD 4 
  unsigned char drive_telegram_4[DRIVE_TELEGRAM_TRANSMIT_SIZE];
  
  //std_msgs::MultiArray variables
  std_msgs::MultiArrayDimension msg_dim;
  std_msgs::UInt8MultiArray msg;
  
  msg_dim.label = "Drive_1_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //Get joint_velocity_command from controller in radians 
  double velocity_rad = -joint_velocity_command_[0];        //! Changing command direction here (CW - CCW)
  
  //Convert to motor rpm (transmission included)
  double velocity_rpm = velocity_rad *DRIVE_01_RATIO * 60 / (2 * M_PI);
  
  //Convert to perc of max speed  
  double velocity_perc = velocity_rpm / DRIVE_01_MAX_MOTOR_VELOCITY;
     
  //Convert to 0x40000000 = 100% of MAX_MOTOR_RPM
  int velocity_raw = velocity_perc * 0x40000000;
    
  //Decompose position from int to 4 * unsigned char
  unsigned char velocity_bytes[4]; 
  
  velocity_bytes[0] = (velocity_raw >> 24) & 0xFF;
  velocity_bytes[1] = (velocity_raw >> 16) & 0xFF;
  velocity_bytes[2] = (velocity_raw >> 8)  & 0xFF;
  velocity_bytes[3] =  velocity_raw & 0xFF;
 
  //Set telegram data
  drive_telegram_4[0] =  0b00000100;   //STW1 low
  drive_telegram_4[1] =  0b11111111;   //STW1 high
  drive_telegram_4[2] =  velocity_bytes[0];   //NSOLL_B1 high
  drive_telegram_4[3] =  velocity_bytes[1];   //NSOLL_B1 low
  drive_telegram_4[4] =  velocity_bytes[2];   //NSOLL_B2_high 
  drive_telegram_4[5] =  velocity_bytes[3];   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0b00000000;  //G1_STW high
  drive_telegram_4[9] =   0b00000000;  //G1_STW low
  drive_telegram_4[10] =  0b00000000;  //G2_STW high
  drive_telegram_4[11] =  0b00000000;  //G2_STW low
  
  //copy telegram data
  msg.data.clear();
  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);
    
  //publish
  pub_drive_01_output_topic.publish(msg); 
  
  
  
  
  msg_dim.label = "Drive_2_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //Get joint_velocity_command from controller in radians 
  velocity_rad = -joint_velocity_command_[1];               //! Changing command direction here (CW - CCW)
  
  //Convert to motor rpm (transmission included)
  velocity_rpm = velocity_rad *DRIVE_02_RATIO * 60 / (2 * M_PI);
  
  //Convert to perc of max speed  
  velocity_perc = velocity_rpm / DRIVE_02_MAX_MOTOR_VELOCITY;
     
  //Convert to 0x40000000 = 100% of MAX_MOTOR_RPM
  velocity_raw = velocity_perc * 0x40000000;
  
  velocity_bytes[0] = (velocity_raw >> 24) & 0xFF;
  velocity_bytes[1] = (velocity_raw >> 16) & 0xFF;
  velocity_bytes[2] = (velocity_raw >> 8)  & 0xFF;
  velocity_bytes[3] =  velocity_raw & 0xFF;
 
  //Set telegram data
  drive_telegram_4[0] =  0b00000100;   //STW1 low
  drive_telegram_4[1] =  0b11111111;   //STW1 high
  drive_telegram_4[2] =  velocity_bytes[0];   //NSOLL_B1 high
  drive_telegram_4[3] =  velocity_bytes[1];   //NSOLL_B1 low
  drive_telegram_4[4] =  velocity_bytes[2];   //NSOLL_B2_high 
  drive_telegram_4[5] =  velocity_bytes[3];   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0b00000000;  //G1_STW high
  drive_telegram_4[9] =   0b00000000;  //G1_STW low
  drive_telegram_4[10] =  0b00000000;  //G2_STW high
  drive_telegram_4[11] =  0b00000000;  //G2_STW low
  
  //copy telegram data
  msg.data.clear();
  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);
    
  //publish
  pub_drive_02_output_topic.publish(msg);
  
  
  
  
  msg_dim.label = "Drive_3_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //Get joint_velocity_command from controller in radians 
  velocity_rad = joint_velocity_command_[2];
  
  //Convert to motor rpm (transmission included)
  velocity_rpm = velocity_rad *DRIVE_03_RATIO * 60 / (2 * M_PI);
  
  //Convert to perc of max speed  
  velocity_perc = velocity_rpm / DRIVE_03_MAX_MOTOR_VELOCITY;
     
  //Convert to 0x40000000 = 100% of MAX_MOTOR_RPM
  velocity_raw = velocity_perc * 0x40000000;
    
  velocity_bytes[0] = (velocity_raw >> 24) & 0xFF;
  velocity_bytes[1] = (velocity_raw >> 16) & 0xFF;
  velocity_bytes[2] = (velocity_raw >> 8)  & 0xFF;
  velocity_bytes[3] =  velocity_raw & 0xFF;
 
  //Set telegram data
  drive_telegram_4[0] =  0b00000100;   //STW1 low
  drive_telegram_4[1] =  0b11111111;   //STW1 high
  drive_telegram_4[2] =  velocity_bytes[0];   //NSOLL_B1 high
  drive_telegram_4[3] =  velocity_bytes[1];   //NSOLL_B1 low
  drive_telegram_4[4] =  velocity_bytes[2];   //NSOLL_B2_high 
  drive_telegram_4[5] =  velocity_bytes[3];   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0b00000000;  //G1_STW high
  drive_telegram_4[9] =   0b00000000;  //G1_STW low
  drive_telegram_4[10] =  0b00000000;  //G2_STW high
  drive_telegram_4[11] =  0b00000000;  //G2_STW low
  
  //copy telegram data
  msg.data.clear();
  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);
    
  //publish
  pub_drive_03_output_topic.publish(msg); 
  
  
  
  
  msg_dim.label = "Drive_4_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //Get joint_velocity_command from controller in radians 
  velocity_rad = joint_velocity_command_[3];
  
  //Convert to motor rpm (transmission included)
  velocity_rpm = velocity_rad * DRIVE_04_RATIO * 60 / (2 * M_PI);
  
  //Convert to perc of max speed  
  velocity_perc = velocity_rpm / DRIVE_04_MAX_MOTOR_VELOCITY;
     
  //Convert to 0x40000000 = 100% of MAX_MOTOR_RPM
  velocity_raw = velocity_perc * 0x40000000;
    
  velocity_bytes[0] = (velocity_raw >> 24) & 0xFF;
  velocity_bytes[1] = (velocity_raw >> 16) & 0xFF;
  velocity_bytes[2] = (velocity_raw >> 8)  & 0xFF;
  velocity_bytes[3] =  velocity_raw & 0xFF;
 
  //Set telegram data
  drive_telegram_4[0] =  0b00000100;   //STW1 low
  drive_telegram_4[1] =  0b11111111;   //STW1 high
  drive_telegram_4[2] =  velocity_bytes[0];   //NSOLL_B1 high
  drive_telegram_4[3] =  velocity_bytes[1];   //NSOLL_B1 low
  drive_telegram_4[4] =  velocity_bytes[2];   //NSOLL_B2_high 
  drive_telegram_4[5] =  velocity_bytes[3];   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0b00000000;  //G1_STW high
  drive_telegram_4[9] =   0b00000000;  //G1_STW low
  drive_telegram_4[10] =  0b00000000;  //G2_STW high
  drive_telegram_4[11] =  0b00000000;  //G2_STW low
  
  //copy telegram data
  msg.data.clear();
  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);
    
  //publish
  pub_drive_04_output_topic.publish(msg);
  
  
  
  
  msg_dim.label = "Drive_5_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //Get joint_velocity_command from controller in radians 
  velocity_rad = -joint_velocity_command_[4];               //! Changing command direction here (CW - CCW)
  
  //Convert to motor rpm (transmission included)
  velocity_rpm = velocity_rad *DRIVE_05_RATIO * 60 / (2 * M_PI);
  
  //Convert to perc of max speed  
  velocity_perc = velocity_rpm / DRIVE_05_MAX_MOTOR_VELOCITY;
     
  //Convert to 0x40000000 = 100% of MAX_MOTOR_RPM
  velocity_raw = velocity_perc * 0x40000000;
  
  velocity_bytes[0] = (velocity_raw >> 24) & 0xFF;
  velocity_bytes[1] = (velocity_raw >> 16) & 0xFF;
  velocity_bytes[2] = (velocity_raw >> 8)  & 0xFF;
  velocity_bytes[3] =  velocity_raw & 0xFF;
 
  //Set telegram data
  drive_telegram_4[0] =  0b00000100;   //STW1 low
  drive_telegram_4[1] =  0b11111111;   //STW1 high
  drive_telegram_4[2] =  velocity_bytes[0];   //NSOLL_B1 high
  drive_telegram_4[3] =  velocity_bytes[1];   //NSOLL_B1 low
  drive_telegram_4[4] =  velocity_bytes[2];   //NSOLL_B2_high 
  drive_telegram_4[5] =  velocity_bytes[3];   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0b00000000;  //G1_STW high
  drive_telegram_4[9] =   0b00000000;  //G1_STW low
  drive_telegram_4[10] =  0b00000000;  //G2_STW high
  drive_telegram_4[11] =  0b00000000;  //G2_STW low
  
  //copy telegram data
  msg.data.clear();
  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);
    
  //publish
  pub_drive_05_output_topic.publish(msg); 
 
  
  
    
  msg_dim.label = "Drive_6_output";
  msg_dim.size = DRIVE_TELEGRAM_TRANSMIT_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);
  
  //Get joint_velocity_command from controller in radians 
  velocity_rad = joint_velocity_command_[5];
  
  //Convert to motor rpm (transmission included)
  velocity_rpm = velocity_rad *DRIVE_06_RATIO * 60 / (2 * M_PI);
  
  //Convert to perc of max speed  
  velocity_perc = velocity_rpm / DRIVE_06_MAX_MOTOR_VELOCITY;
     
  //Convert to 0x40000000 = 100% of MAX_MOTOR_RPM
  velocity_raw = velocity_perc * 0x40000000;
    
  velocity_bytes[0] = (velocity_raw >> 24) & 0xFF;
  velocity_bytes[1] = (velocity_raw >> 16) & 0xFF;
  velocity_bytes[2] = (velocity_raw >> 8)  & 0xFF;
  velocity_bytes[3] =  velocity_raw & 0xFF;
 
  //Set telegram data
  drive_telegram_4[0] =  0b00000100;   //STW1 low
  drive_telegram_4[1] =  0b11111111;   //STW1 high
  drive_telegram_4[2] =  velocity_bytes[0];   //NSOLL_B1 high
  drive_telegram_4[3] =  velocity_bytes[1];   //NSOLL_B1 low
  drive_telegram_4[4] =  velocity_bytes[2];   //NSOLL_B2_high 
  drive_telegram_4[5] =  velocity_bytes[3];   //NSOLL_B2_low
  drive_telegram_4[6] =  0b00000000;   //STW2 high
  drive_telegram_4[7] =  0b00000000;   //STW2 low

  drive_telegram_4[8] =   0b00000000;  //G1_STW high
  drive_telegram_4[9] =   0b00000000;  //G1_STW low
  drive_telegram_4[10] =  0b00000000;  //G2_STW high
  drive_telegram_4[11] =  0b00000000;  //G2_STW low
  
  //copy telegram data
  msg.data.clear();
  for(int i = 0;  i < DRIVE_TELEGRAM_TRANSMIT_SIZE; i++)
    msg.data.push_back(drive_telegram_4[i]);
    
  //publish
  pub_drive_06_output_topic.publish(msg); 
}

void SefRoboterHWInterface::enforceLimits(ros::Duration &period)
{

  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  //vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

}

}  // namespace
