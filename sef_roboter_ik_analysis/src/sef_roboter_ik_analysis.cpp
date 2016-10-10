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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Frantisek Durovsky */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "sef_roboter_ik_analysis");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //Create Robot kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  std::vector<double> joint_values;

  for(int i = 0; i < 10000; i++)
  {
    // Forward Kinematics
    kinematic_state->setToRandomPositions(joint_model_group);
    kinematic_state->enforceBounds();
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_tool");

    // Inverse Kinematics
    ros::Time ik_begin = ros::Time::now();
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 5, 0.1);
    ros::Time ik_end = ros::Time::now();
    ros::Duration ik_duration = ik_end - ik_begin;

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      ROS_INFO("Joints: [ %f, %f, %f, %f, %f, %f], duration: %f", joint_values[0],
                                                    joint_values[1],
                                                    joint_values[2],
                                                    joint_values[3],
                                                    joint_values[4],
                                                    joint_values[5],
                                                    (double)ik_duration.nsec/(double)1000000000);

    }
    else
        ROS_INFO("Did not find IK solution");
  }
  ros::shutdown();
  return 0;
}

