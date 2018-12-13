/**********
***********************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
using namespace visualization_msgs;

float table1_position[3];
float table2_position[3];
float object_starting_position[3];
float object_finish_position[3];

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_1_joint";
  posture.joint_names[1] = "finger_1_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = -0.01;
  posture.points[0].positions[1] = 0.01;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_1_joint";
  posture.joint_names[1] = "finger_1_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group , int i )
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
      std::vector<moveit_msgs::Grasp> grasps;
      grasps.resize(1);
  // Setting grasp pose
      grasps[i].grasp_pose.header.frame_id = "base_link";
      grasps[i].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI , 0); // OK
  // M_PI/2, 0, M_PI/2
  //0, M_PI , 0 dole hlavou gripper +x suradnice
      grasps[i].grasp_pose.pose.position.x = object_starting_position[0];
      grasps[i].grasp_pose.pose.position.y = object_starting_position[1];
      grasps[i].grasp_pose.pose.position.z = object_starting_position[2]+0.16;
//-0.15
      for (int i =0 ; i<3 ; i++){
       ROS_INFO("Test11 %f \n",table1_position[i]);
       }

      for (int i =0 ; i<3 ; i++){
       ROS_INFO("Test22 %f \n",table2_position[i]);
     }
  // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[i].pre_grasp_approach.direction.header.frame_id = "base_link";
    /* Direction is set as positive x axis */
    grasps[i].pre_grasp_approach.direction.vector.z = -0.2;
    grasps[i].pre_grasp_approach.min_distance = 0.1;
    grasps[i].pre_grasp_approach.desired_distance = 0.2;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[i].post_grasp_retreat.direction.header.frame_id = "base_link";
    /* Direction is set as positive z axis */
    grasps[i].post_grasp_retreat.direction.vector.z = 0.2;
    grasps[i].post_grasp_retreat.min_distance = 0.1;
    grasps[i].post_grasp_retreat.desired_distance = 0.2;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[i].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[i].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group , int i)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[i].place_pose.header.frame_id = "base_link";
  place_location[i].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

// uhol pozy ulozenia sa urcuje nasledovne
// uhol uchopu plus modifikacia !!!
  /* While placing it is the exact location of the center of the object. */
  place_location[i].place_pose.pose.position.x = object_finish_position[0];
  place_location[i].place_pose.pose.position.y = object_finish_position[1];
  place_location[i].place_pose.pose.position.z = object_finish_position[2];

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[i].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[i].pre_place_approach.direction.vector.z = -0.2;
  place_location[i].pre_place_approach.min_distance = 0.1;
  place_location[i].pre_place_approach.desired_distance = 0.2;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[i].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[i].post_place_retreat.direction.vector.z = 0.2;
  place_location[i].post_place_retreat.min_distance = 0.1;
  place_location[i].post_place_retreat.desired_distance = 0.2;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[i].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL

}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);

  collision_objects[0].primitive_poses[0].position.x = table1_position[0];
  collision_objects[0].primitive_poses[0].position.y = table1_position[1];
  collision_objects[0].primitive_poses[0].position.z = table1_position[2];

  collision_objects[0].primitive_poses[0].orientation.x = 0;
  collision_objects[0].primitive_poses[0].orientation.y = 0;
  collision_objects[0].primitive_poses[0].orientation.z = 0;
  collision_objects[0].primitive_poses[0].orientation.w = 1;

  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);

  collision_objects[1].primitive_poses[0].position.x = table2_position[0];
  collision_objects[1].primitive_poses[0].position.y = table2_position[1];
  collision_objects[1].primitive_poses[0].position.z = table2_position[2];

  collision_objects[1].primitive_poses[0].orientation.x = 0;
  collision_objects[1].primitive_poses[0].orientation.y = 0;
  collision_objects[1].primitive_poses[0].orientation.z = 0;
  collision_objects[1].primitive_poses[0].orientation.w = 1;

  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.02;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = object_starting_position[0];
  collision_objects[2].primitive_poses[0].position.y = object_starting_position[1];
  collision_objects[2].primitive_poses[0].position.z = object_starting_position[2];

  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void PositionInputCallback(const InteractiveMarkerFeedback& msg){
  if(msg.menu_entry_id == 1){

    table1_position[0] = msg.pose.position.x;
    table1_position[1] = msg.pose.position.y;
    table1_position[2] = msg.pose.position.z;

    object_starting_position[0] = msg.pose.position.x;
    object_starting_position[1] = msg.pose.position.y;
    object_starting_position[2] = msg.pose.position.z + 0.21;



    for (int i =0 ; i<3 ; i++){
     ROS_INFO("Test1 %f \n",table1_position[i]);
     }

     for (int i =0 ; i<3 ; i++){
      ROS_INFO("Test2 %f \n",table2_position[i]);
    }
  }
  if (msg.menu_entry_id == 2)
  {

    table2_position[0] = msg.pose.position.x;
    table2_position[1] = msg.pose.position.y;
    table2_position[2] = msg.pose.position.z;

    object_finish_position[0] = msg.pose.position.x;
    object_finish_position[1] = msg.pose.position.y;
    object_finish_position[2] = msg.pose.position.z + 0.21;

    for (int i =0 ; i<3 ; i++){
     ROS_INFO("Test1 %f \n",table1_position[i]);
     }

    for (int i =0 ; i<3 ; i++){
     ROS_INFO("Test2 %f \n",table2_position[i]);
   }

  }
   // ROS_INFO("Test2 %d",&msg.pose.position.x);
  ROS_INFO("Slucham");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sef_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber sub = nh.subscribe("/basic_controls/feedback", 10 , PositionInputCallback);

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(100.0);



while(ros::ok()){
int i = 0 ;
  ROS_INFO_STREAM("Start planning nigguh");
  // ros::WallDuration(6.0).sleep();
  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  // ros::WallDuration(1.0).sleep();

  pick(group, i);

  ros::WallDuration(0.5).sleep();

  place(group, i);
 i ++;
 ROS_INFO("%d",i);
}
ros::waitForShutdown();
  return 0;
}
