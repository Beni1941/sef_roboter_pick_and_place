/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CONTROL_PANEL
#define CONTROL_PANEL

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include<QSlider>
#include<QWidget>
#include<QPushButton>
#include<QAbstractSlider>
#include<QString>
#include<QGroupBox>
#include<QHBoxLayout>
#include<QLabel>
#include<QGridLayout>
#include<QLayout>
#include<QLCDNumber>
#include<QSpinBox>
#include <QObject>
#include <rviz/panel.h>
#include <QScrollBar>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "std_msgs/String.h"

#include <sstream>
#endif



namespace control_plugin
{






class ControlPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  ControlPanel( QWidget* parent = 0 );

void CreateJointGroups();

void ConnectJointGroups();

void MovementInit();

void PlanningInit();



public Q_SLOTS:

void Update_Numbers(int valueChanged);
void Home();
void GoToGoal1();
void pokus(const std_msgs::String &chatter);
void GoToGoal2();

Q_SIGNALS:



public:
  QSlider *joint1_slider_goal1;
  QSlider *joint2_slider_goal1;
  QSlider *joint3_slider_goal1;
  QSlider *joint4_slider_goal1;
  QSlider *joint5_slider_goal1;
  QSlider *joint6_slider_goal1;

  QSlider *joint1_slider_goal2;
  QSlider *joint2_slider_goal2;
  QSlider *joint3_slider_goal2;
  QSlider *joint4_slider_goal2;
  QSlider *joint5_slider_goal2;
  QSlider *joint6_slider_goal2;

  QLabel *label1;
  QLabel *value1;
  QLabel *value2;


  QLabel *joint1_label;
  QLabel *joint2_label;
  QLabel *joint3_label;
  QLabel *joint4_label;
  QLabel *joint5_label;
  QLabel *joint6_label;

  QLabel *max_value_joint1;
  QLabel *max_value_joint2;
  QLabel *max_value_joint3;
  QLabel *max_value_joint4;
  QLabel *max_value_joint5;
  QLabel *max_value_joint6;

  QLabel *min_value_joint1;
  QLabel *min_value_joint2;
  QLabel *min_value_joint3;
  QLabel *min_value_joint4;
  QLabel *min_value_joint5;
  QLabel *min_value_joint6;


  QLCDNumber *joint1_indicator_goal1;
  QLCDNumber *joint2_indicator_goal1;
  QLCDNumber *joint3_indicator_goal1;
  QLCDNumber *joint4_indicator_goal1;
  QLCDNumber *joint5_indicator_goal1;
  QLCDNumber *joint6_indicator_goal1;

  QLCDNumber *joint1_indicator_goal2;
  QLCDNumber *joint2_indicator_goal2;
  QLCDNumber *joint3_indicator_goal2;
  QLCDNumber *joint4_indicator_goal2;
  QLCDNumber *joint5_indicator_goal2;
  QLCDNumber *joint6_indicator_goal2;

  QSpinBox *joint1_spinbox;
  QSpinBox *joint2_spinbox;
  QSpinBox *joint3_spinbox;
  QSpinBox *joint4_spinbox;
  QSpinBox *joint5_spinbox;
  QSpinBox *joint6_spinbox;

  QPushButton *home;
  QPushButton *goal1;
  QPushButton *goal2;

  QGridLayout *grid;

protected :
  float goal1_position[6];
  float goal2_position[6];
  int argc;
  char **argv;
  ros::Publisher pub_goal_position ;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  sensor_msgs::JointState joint_state ;
 };


}// end namespace rviz_plugin_tutorials
#endif // TELEOP_PANEL_H
