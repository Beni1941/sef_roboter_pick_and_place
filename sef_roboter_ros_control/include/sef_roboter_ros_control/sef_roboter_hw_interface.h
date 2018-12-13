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
   Desc:   ros_control hardware interface for the sef_roboter_sr25
*/

#ifndef SEF_ROBOTER_HW_INTERFACE_H
#define SEF_ROBOTER_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

//Headers required for communication with siemens_cp1616 topics
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

//Headers required for direct homing command
#include <trajectory_msgs/JointTrajectory.h>
#include <sef_roboter_msgs/reference_joint.h>
#include <sef_roboter_msgs/homing.h>

namespace sef_roboter_hw_control
{

/// \brief Hardware interface for a robot
class SefRoboterHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  SefRoboterHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /**
   * \brief Destructor
   */
  ~SefRoboterHWInterface();

  /** \brief Send initialization telegrams to all drives. */
  void initDrives();

  /** \brief Callback function for homing service. */
  bool homingCallback(sef_roboter_msgs::homing::Request &req,
                      sef_roboter_msgs::homing::Response &res);

  /** \brief Callback function for reference joint service. */
  bool referenceJointCallback(sef_roboter_msgs::reference_joint::Request &req,
                              sef_roboter_msgs::reference_joint::Response &res);

  /** \brief Callback function to process drive 1 states */
  void drive01Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data);
  
  /** \brief Callback function to process drive 2 states */
  void drive02Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data);
  
  /** \brief Callback function to process drive 3 states */
  void drive03Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data);
  
  /** \brief Callback function to process drive 4 states */
  void drive04Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data);
  
  /** \brief Callback function to process drive 5 states */
  void drive05Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data);
  
  /** \brief Callback function to process drive 6 states */
  void drive06Callback(const std_msgs::UInt8MultiArray::ConstPtr &callback_data);
  
  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

private:
  ros::Publisher pub_drive_01_output_topic;
  ros::Publisher pub_drive_02_output_topic;
  ros::Publisher pub_drive_03_output_topic;
  ros::Publisher pub_drive_04_output_topic;
  ros::Publisher pub_drive_05_output_topic;
  ros::Publisher pub_drive_06_output_topic;
  ros::Publisher pub_trajectory;

  const int NUM_OF_JOINTS = 6;
  const int RESOLVER_RESOLUTION = 2048;

  const int CU_TELEGRAM_TRANSMIT_SIZE = 4;
  const int CU_TELEGRAM_RECEIVE_SIZE = 4;

  const int DRIVE_TELEGRAM_TRANSMIT_SIZE = 14;
  const int DRIVE_TELEGRAM_RECEIVE_SIZE = 32;
  
  //Drive 01 consts & variables
  const double DRIVE_01_MAX_MOTOR_VELOCITY = 3000;	      		
  const double DRIVE_01_RATIO = 80;
  
  double drive_01_actual_position_rad;
  double drive_01_actual_velocity_rad_s;
  bool drive_01_referencing_flag;

  //Drive 02 consts & variables
  const double DRIVE_02_MAX_MOTOR_VELOCITY = 3000;	      		
  const double DRIVE_02_RATIO = 120;
  
  double drive_02_actual_position_rad;
  double drive_02_actual_velocity_rad_s;
  bool drive_02_referencing_flag;
  
  //Drive 03 consts & variables
  const double DRIVE_03_MAX_MOTOR_VELOCITY = 3000;	      		
  const double DRIVE_03_RATIO = 108;
  
  double drive_03_actual_position_rad;
  double drive_03_actual_velocity_rad_s;
  bool drive_03_referencing_flag;
  
  //Drive 04 consts & variables
  const double DRIVE_04_MAX_MOTOR_VELOCITY = 6000;	      		
  const double DRIVE_04_RATIO = 76.5;
  
  double drive_04_actual_position_rad;
  double drive_04_actual_velocity_rad_s;
  bool drive_04_referencing_flag;
  
  //Drive 05 consts & variables
  const double DRIVE_05_MAX_MOTOR_VELOCITY = 6000;	      		
  const double DRIVE_05_RATIO = 100.5;
  
  double drive_05_actual_position_rad;
  double drive_05_actual_velocity_rad_s;
  bool drive_05_referencing_flag;
  
  //Drive 06 consts & variables
  const double DRIVE_06_MAX_MOTOR_VELOCITY = 6000;	      		
  const double DRIVE_06_RATIO = 92.2;
  
  double drive_06_actual_position_rad;
  double drive_06_actual_velocity_rad_s;
  bool drive_06_referencing_flag;
    
};  // class

}  // namespace

#endif
