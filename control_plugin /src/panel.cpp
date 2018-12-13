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

#include "panel.h"

namespace control_plugin
{
ControlPanel::ControlPanel( QWidget* parent )
  : rviz::Panel( parent )
{


 this->setWindowTitle("SR25 Joint Publisher");

 for (int i = 0; i < 7 ; i++)
    {
        goal1_position[i] = 0;
        goal2_position[i] = 0;
    }


CreateJointGroups();
ConnectJointGroups();
MovementInit();
}


void ControlPanel::MovementInit(){
pub_goal_position = nh.advertise<sensor_msgs::JointState>("test", 1);

};
void ControlPanel::pokus(const std_msgs::String &chatter){
  ROS_INFO("Prijmam chatter");
}

void ControlPanel::CreateJointGroups(){

// sub = nh.subscribe("chatter", 1, pokus(chatter));
  value1 = new QLabel("Goal1 values");
  value2 = new QLabel("Goal2 values");

  home = new QPushButton("HOME",this);

  goal1 = new QPushButton("Go to goal1",this);

  goal2 = new QPushButton("Go to goal2",this);

  joint1_slider_goal1 = new QSlider(Qt::Horizontal ,this);
  joint2_slider_goal1 = new QSlider(Qt::Horizontal, this);
  joint3_slider_goal1 = new QSlider(Qt::Horizontal ,this);
  joint4_slider_goal1 = new QSlider(Qt::Horizontal, this);
  joint5_slider_goal1 = new QSlider(Qt::Horizontal ,this);
  joint6_slider_goal1 = new QSlider(Qt::Horizontal, this);

  joint1_slider_goal1->setRange(-100,100);
  joint2_slider_goal1->setRange(-50,150);
  joint3_slider_goal1->setRange(-150,50);
  joint4_slider_goal1->setRange(-250,250);
  joint5_slider_goal1->setRange(-160,160);
  joint6_slider_goal1->setRange(-250,250);

  joint1_slider_goal1->setValue(0);
  joint2_slider_goal1->setValue(0);
  joint3_slider_goal1->setValue(0);
  joint4_slider_goal1->setValue(0);
  joint5_slider_goal1->setValue(0);
  joint6_slider_goal1->setValue(0);

  joint1_slider_goal1->setMinimumWidth(100);


  joint1_slider_goal1->setSingleStep(0.1);

  joint1_slider_goal2 = new QSlider(Qt::Horizontal ,this);
  joint2_slider_goal2 = new QSlider(Qt::Horizontal, this);
  joint3_slider_goal2 = new QSlider(Qt::Horizontal ,this);
  joint4_slider_goal2 = new QSlider(Qt::Horizontal, this);
  joint5_slider_goal2 = new QSlider(Qt::Horizontal ,this);
  joint6_slider_goal2 = new QSlider(Qt::Horizontal, this);

  joint1_slider_goal2->setRange(-100,100);
  joint2_slider_goal2->setRange(-50,150);
  joint3_slider_goal2->setRange(-150,50);
  joint4_slider_goal2->setRange(-250,250);
  joint5_slider_goal2->setRange(-160,160);
  joint6_slider_goal2->setRange(-250,250);

  joint1_slider_goal2->setValue(0);
  joint2_slider_goal2->setValue(0);
  joint3_slider_goal2->setValue(0);
  joint4_slider_goal2->setValue(0);
  joint5_slider_goal2->setValue(0);
  joint6_slider_goal2->setValue(0);

  joint1_slider_goal2->setMinimumWidth(100);

  max_value_joint1 = new QLabel("1");
  max_value_joint2 = new QLabel("1,5");
  max_value_joint3 = new QLabel("0.5");
  max_value_joint4 = new QLabel("2.5");
  max_value_joint5 = new QLabel("1.6");
  max_value_joint6 = new QLabel("2.5");

  min_value_joint1 = new QLabel("-1");
  min_value_joint2 = new QLabel("-0.5");
  min_value_joint3 = new QLabel("-1.5");
  min_value_joint4 = new QLabel("-2.5");
  min_value_joint5 = new QLabel("-1.6");
  min_value_joint6 = new QLabel("-2.5");

  joint1_label = new QLabel("Joint1");
  joint2_label = new QLabel("Joint2");
  joint3_label = new QLabel("Joint3");
  joint4_label = new QLabel("Joint4");
  joint5_label = new QLabel("Joint5");
  joint6_label = new QLabel("Joint6");

  joint1_indicator_goal1 = new QLCDNumber;
  joint2_indicator_goal1 = new QLCDNumber;
  joint3_indicator_goal1 = new QLCDNumber;
  joint4_indicator_goal1 = new QLCDNumber;
  joint5_indicator_goal1 = new QLCDNumber;
  joint6_indicator_goal1 = new QLCDNumber;

  joint1_indicator_goal2 = new QLCDNumber;
  joint2_indicator_goal2 = new QLCDNumber;
  joint3_indicator_goal2 = new QLCDNumber;
  joint4_indicator_goal2 = new QLCDNumber;
  joint5_indicator_goal2 = new QLCDNumber;
  joint6_indicator_goal2 = new QLCDNumber;

  joint2_indicator_goal1 ->display("0.0");
  joint3_indicator_goal1 ->display("0.0");

  joint2_indicator_goal2 ->display("0.0");
  joint3_indicator_goal2 ->display("0.0");

  grid = new QGridLayout(this);
  grid->setHorizontalSpacing(20);
  grid->setVerticalSpacing(10);

  grid->addWidget(value1, 0, 1);

  grid->addWidget(value2, 0, 5);

  grid->addWidget(joint1_label, 1, 0);
  grid->addWidget(joint2_label, 2, 0);
  grid->addWidget(joint3_label, 3, 0);
  grid->addWidget(joint4_label, 4, 0);
  grid->addWidget(joint5_label, 5, 0);
  grid->addWidget(joint6_label, 6, 0);

  grid->addWidget(joint1_indicator_goal1, 1 , 1);
  grid->addWidget(joint2_indicator_goal1, 2 , 1);
  grid->addWidget(joint3_indicator_goal1, 3 , 1);
  grid->addWidget(joint4_indicator_goal1, 4 , 1);
  grid->addWidget(joint5_indicator_goal1, 5 , 1);
  grid->addWidget(joint6_indicator_goal1, 6 , 1);

  grid->addWidget(min_value_joint1, 1,2);
  grid->addWidget(min_value_joint2, 2,2);
  grid->addWidget(min_value_joint3, 3,2);
  grid->addWidget(min_value_joint4, 4,2);
  grid->addWidget(min_value_joint5, 5,2);
  grid->addWidget(min_value_joint6, 6,2);

  grid->addWidget(joint1_slider_goal1 , 1,3);
  grid->addWidget(joint2_slider_goal1 , 2,3);
  grid->addWidget(joint3_slider_goal1 , 3,3);
  grid->addWidget(joint4_slider_goal1 , 4,3);
  grid->addWidget(joint5_slider_goal1 , 5,3);
  grid->addWidget(joint6_slider_goal1 , 6,3);

  grid->addWidget(max_value_joint1, 1,4);
  grid->addWidget(max_value_joint2, 2,4);
  grid->addWidget(max_value_joint3, 3,4);
  grid->addWidget(max_value_joint4, 4,4);
  grid->addWidget(max_value_joint5, 5,4);
  grid->addWidget(max_value_joint6, 6,4);

  grid->addWidget(joint1_indicator_goal2 , 1 , 5);
  grid->addWidget(joint2_indicator_goal2 , 2 , 5);
  grid->addWidget(joint3_indicator_goal2 , 3 , 5);
  grid->addWidget(joint4_indicator_goal2 , 4 , 5);
  grid->addWidget(joint5_indicator_goal2 , 5 , 5);
  grid->addWidget(joint6_indicator_goal2 , 6 , 5);

  grid->addWidget(min_value_joint1, 1, 6);
  grid->addWidget(min_value_joint2, 2, 6);
  grid->addWidget(min_value_joint3, 3, 6);
  grid->addWidget(min_value_joint4, 4, 6);
  grid->addWidget(min_value_joint5, 5, 6);
  grid->addWidget(min_value_joint6, 6, 6);

  grid->addWidget(joint1_slider_goal2, 1, 7);
  grid->addWidget(joint2_slider_goal2, 2, 7);
  grid->addWidget(joint3_slider_goal2, 3, 7);
  grid->addWidget(joint4_slider_goal2, 4, 7);
  grid->addWidget(joint5_slider_goal2, 5, 7);
  grid->addWidget(joint6_slider_goal2, 6, 7);

  grid->addWidget(max_value_joint1, 1, 8);
  grid->addWidget(max_value_joint2, 2, 8);
  grid->addWidget(max_value_joint3, 3, 8);
  grid->addWidget(max_value_joint4, 4, 8);
  grid->addWidget(max_value_joint5, 5, 8);
  grid->addWidget(max_value_joint6, 6, 8);

  grid->addWidget(home, 7,0);
  grid->addWidget(goal1,7,1);
  grid->addWidget(goal2,7,5);

  setLayout(grid);

};


//ak spajas signal cisto len na dalsiu funkciu (slot), tak das ...this, ...
void ControlPanel::Update_Numbers(int valueChanged)
{
goal1_position[1] = (float)(joint1_slider_goal1->value())/100;
goal1_position[2] = (float)(joint2_slider_goal1->value())/100;
goal1_position[3] = (float)(joint3_slider_goal1->value())/100;
goal1_position[4] = (float)(joint4_slider_goal1->value())/100;
goal1_position[5] = (float)(joint5_slider_goal1->value())/100;
goal1_position[6] = (float)(joint6_slider_goal1->value())/100;

goal2_position[1] = (float)(joint1_slider_goal2->value())/100;
goal2_position[2] = (float)(joint2_slider_goal2->value())/100;
goal2_position[3] = (float)(joint3_slider_goal2->value())/100;
goal2_position[4] = (float)(joint4_slider_goal2->value())/100;
goal2_position[5] = (float)(joint5_slider_goal2->value())/100;
goal2_position[6] = (float)(joint6_slider_goal2->value())/100;

joint1_indicator_goal1->display(double(goal1_position[1]));
joint2_indicator_goal1->display(double(goal1_position[2]));
joint3_indicator_goal1->display(double(goal1_position[3]));
joint4_indicator_goal1->display(double(goal1_position[4]));
joint5_indicator_goal1->display(double(goal1_position[5]));
joint6_indicator_goal1->display(double(goal1_position[6]));

joint1_indicator_goal2->display(double(goal2_position[1]));
joint2_indicator_goal2->display(double(goal2_position[2]));
joint3_indicator_goal2->display(double(goal2_position[3]));
joint4_indicator_goal2->display(double(goal2_position[4]));
joint5_indicator_goal2->display(double(goal2_position[5]));
joint6_indicator_goal2->display(double(goal2_position[6]));

joint_state.header.stamp = ros::Time::now();
joint_state.name.resize(7);
joint_state.position.resize(7);
joint_state.name[1] ="joint_1";
joint_state.position[1] = goal1_position[1];
joint_state.name[2] ="joint_2";
joint_state.position[2] = goal1_position[2];
joint_state.name[3] ="joint_3";
joint_state.position[3] = goal1_position[3];
joint_state.name[4] ="joint_4";
joint_state.position[4] = goal1_position[4];
joint_state.name[5] ="joint_5";
joint_state.position[5] = goal1_position[5];
joint_state.name[6] ="joint_6";
joint_state.position[6] = goal1_position[6];
pub_goal_position.publish(joint_state);

};


void ControlPanel::Home()
{
    goal1_position[1] = 0;
    goal1_position[2] = 0;
    goal1_position[3] = 0;
    goal1_position[4] = 0;
    goal1_position[5] = 0;
    goal1_position[6] = 0;

    goal2_position[1] = 0;
    goal2_position[2] = 0;
    goal2_position[3] = 0;
    goal2_position[4] = 0;
    goal2_position[5] = 0;
    goal2_position[6] = 0;

    joint1_indicator_goal1->display(double(goal1_position[1]));
    joint2_indicator_goal1->display(double(goal1_position[2]));
    joint3_indicator_goal1->display(double(goal1_position[3]));
    joint4_indicator_goal1->display(double(goal1_position[4]));
    joint5_indicator_goal1->display(double(goal1_position[5]));
    joint6_indicator_goal1->display(double(goal1_position[6]));

    joint1_indicator_goal2->display(double(goal2_position[1]));
    joint2_indicator_goal2->display(double(goal2_position[2]));
    joint3_indicator_goal2->display(double(goal2_position[3]));
    joint4_indicator_goal2->display(double(goal2_position[4]));
    joint5_indicator_goal2->display(double(goal2_position[5]));
    joint6_indicator_goal2->display(double(goal2_position[6]));

    joint1_slider_goal1->setValue(0);
    joint2_slider_goal1->setValue(0);
    joint3_slider_goal1->setValue(0);
    joint4_slider_goal1->setValue(0);
    joint5_slider_goal1->setValue(0);
    joint6_slider_goal1->setValue(0);

    joint1_slider_goal2->setValue(0);
    joint2_slider_goal2->setValue(0);
    joint3_slider_goal2->setValue(0);
    joint4_slider_goal2->setValue(0);
    joint5_slider_goal2->setValue(0);
    joint6_slider_goal2->setValue(0);

    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(7);
    joint_state.position.resize(7);
    joint_state.name[1] ="joint_1";
    joint_state.position[1] = 0.0;
    joint_state.name[2] ="joint_2";
    joint_state.position[2] = 0.0;
    joint_state.name[3] ="joint_3";
    joint_state.position[3] = 0.0;
    joint_state.name[4] ="joint_4";
    joint_state.position[4] = 0.0;
    joint_state.name[5] ="joint_5";
    joint_state.position[5] = 0.0;
    joint_state.name[6] ="joint_6";
    joint_state.position[6] = 0.0;
    pub_goal_position.publish(joint_state);


ROS_INFO("Going home.");

}



void ControlPanel::ConnectJointGroups()
{

QObject::connect(joint1_slider_goal1,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint2_slider_goal1,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint3_slider_goal1,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint4_slider_goal1,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint5_slider_goal1,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint6_slider_goal1,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));

QObject::connect(joint1_slider_goal2,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint2_slider_goal2,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint3_slider_goal2,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint4_slider_goal2,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint5_slider_goal2,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));
QObject::connect(joint6_slider_goal2,SIGNAL(valueChanged(int)),this,SLOT(Update_Numbers(int)));

QObject::connect(home,SIGNAL(clicked()),this,SLOT(Home()));

QObject::connect(goal1 , SIGNAL(clicked()),this,SLOT(GoToGoal1()));
QObject::connect(goal2 , SIGNAL(clicked()),this,SLOT(GoToGoal2()));
};
//po stlaceni tlacidla Rviz mrzne.
void ControlPanel::GoToGoal1(){
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(7);
  joint_state.position.resize(7);
  joint_state.name[1] ="joint_1";
  joint_state.position[1] = goal1_position[1];
  joint_state.name[2] ="joint_2";
  joint_state.position[2] = goal1_position[2];
  joint_state.name[3] ="joint_3";
  joint_state.position[3] = goal1_position[3];
  joint_state.name[4] ="joint_4";
  joint_state.position[4] = goal1_position[4];
  joint_state.name[5] ="joint_5";
  joint_state.position[5] = goal1_position[5];
  joint_state.name[6] ="joint_6";
  joint_state.position[6] = goal1_position[6];
  pub_goal_position.publish(joint_state);
  PlanningInit();

};

void ControlPanel::GoToGoal2(){
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(7);
  joint_state.position.resize(7);
  joint_state.name[1] ="joint_1";
  joint_state.position[1] = goal2_position[1];
  joint_state.name[2] ="joint_2";
  joint_state.position[2] = goal2_position[2];
  joint_state.name[3] ="joint_3";
  joint_state.position[3] = goal2_position[3];
  joint_state.name[4] ="joint_4";
  joint_state.position[4] = goal2_position[4];
  joint_state.name[5] ="joint_5";
  joint_state.position[5] = goal2_position[5];
  joint_state.name[6] ="joint_6";
  joint_state.position[6] = goal2_position[6];
  pub_goal_position.publish(joint_state);
  PlanningInit();
};

void ControlPanel::PlanningInit(){
//system("roslaunch control_plugin_moveit_node node.launch");
ros::init(argc, argv, "control_plugin_moveit");
ros::AsyncSpinner spinner(1);
 // ros::Subscriber joint_states_user_input = node_handle.subscribe("test",1, joint_state_message_recieved);
 spinner.start();
// BEGIN_TUTORIAL
//
// Setup
// ^^^^^
//
// MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
// the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
// are used interchangably.
static const std::string PLANNING_GROUP = "arm";
// The :move_group_interface:`MoveGroup` class can be easily
// setup using just the name of the planning group you would like to control and plan for.
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
// We will use the :planning_scene_interface:`PlanningSceneInterface`
// class to add and remove collision objects in our "virtual world" scene
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
// Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      // Getting Basic Information
      // ^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      // We can print the name of the reference frame for this robot.
      ROS_INFO_NAMED("control_plugin", "Reference frame: %s", move_group.getPlanningFrame().c_str());
      // We can also print the name of the end-effector link for this group.
      ROS_INFO_NAMED("control_plugin", "End effector link: %s", move_group.getEndEffectorLink().c_str());
      // Start the demo
       moveit::planning_interface::MoveGroupInterface::Plan my_plan;
       bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
       ROS_INFO_NAMED("control_plugin", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
       // Planning to a joint-space goal
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      // Let's set a joint space goal and move towards it.  This will replace the
      // pose target we set above.
      //
      // To start, we'll create an pointer that references the current robot's state.
      // RobotState is the object that contains all the current position/velocity/acceleration data.
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
      joint_group_positions[0] = joint_state.position[1];  // radians
      joint_group_positions[1] = joint_state.position[2];
      joint_group_positions[2] = joint_state.position[3];
      joint_group_positions[3] = joint_state.position[4];
      joint_group_positions[4] = joint_state.position[5];
      joint_group_positions[5] = joint_state.position[6];

      move_group.setJointValueTarget(joint_group_positions);

      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("control_plugin", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
      move_group.move();
};
} // end namespace control_plugin
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::ControlPanel,rviz::Panel)
// PLUGINLIB_EXPORT_CLASS(pluginlib::ClassLoader<planning_interface::PlannerManager> , rviz::Panel)
// END_TUTORIAL
