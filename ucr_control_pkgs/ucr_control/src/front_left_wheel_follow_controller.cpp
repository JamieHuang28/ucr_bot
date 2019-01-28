/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jamie Huang
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
 *   * Neither the name of the PAL Robotics nor the names of its
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
 
 /*
  *autor: Jamie Huang
  */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

static std::string g_rear_drive_joint_name;
static double g_rear_drive_joint_velocity;
static std::string g_steer_joint_name;
static double g_steer_joint_velocity;
static double g_steer_joint_position;

// everytime when command available, update required velocity, it is additional
void CommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  g_rear_drive_joint_velocity = msg->linear.x;// get linear velicity
  ROS_DEBUG_STREAM("CommandCallback is called, and arg = " << *msg << " , result = " << g_rear_drive_joint_velocity);
}

// to follow the rear wheel taking steer_velocity into consideration
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int i;
  for(i = 0; i < msg->velocity.size(); ++i)
  {
    if (msg->name[i] == g_steer_joint_name)
    {
      g_steer_joint_velocity = msg->velocity[i];// caution: angular velocity is not true
      g_steer_joint_velocity = g_steer_joint_velocity*4096/60;// correct the scale from supposed transmission
      break;
    }
  }
  if(i == msg->velocity.size())
    ROS_DEBUG_STREAM("no velocity of joint \'" << g_steer_joint_name << "\' is available." );
  
  for(i = 0; i < msg->velocity.size(); ++i)
  {
    if (msg->name[i] == g_steer_joint_name)
    {
      g_steer_joint_position = msg->position[i];
      break;
    }
  }
  if(i == msg->velocity.size())
    ROS_DEBUG_STREAM("no velocity of joint \'" << g_steer_joint_name << "\' is available." );
  
  /*for(i = 0; i < msg->velocity.size(); ++i)
  {
    // if velocity of rear_drive_joint available, update required velocity
    if (msg->name[i] == g_rear_drive_joint_name)
    {
      g_rear_drive_joint_velocity = msg->velocity[i]*wheel_radius;// turn into linear velocity
      break;
    }
  }
  if(i == msg->velocity.size())
    ROS_DEBUG_STREAM("no velocity of joint \'" << g_rear_drive_joint_name << "\' is available." );*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_left_wheel_follow_controller");
  
  ros::NodeHandle nh("~");
  
  ros::Publisher velocity_pub = nh.advertise<std_msgs::Float64>("velocity", 1);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/steer_drive_controller/cmd_vel", 1, CommandCallback);
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, JointStateCallback);
  
  int publish_rate;
  double wheel_seperation_w;// between left wheel and right wheel
  double wheel_radius;
  
  // get parameters
  if(!nh.param<int>("publish_rate", publish_rate, 50))
  {
    ROS_WARN("failed to get parameter of publish_rate, use %d as default value.", 50);
  }
  if(!nh.param<double>("wheel_seperation_w", wheel_seperation_w, 1.0))
  {
    ROS_WARN("failed to get parameter of wheel_seperation_w, use %.2f as default value.", 1.0);
  }
  if(!nh.param<double>("wheel_radius", wheel_radius, 0.5))
  {
    ROS_WARN("failed to get parameter of wheel_radius, use %.2f as default value.", 0.5);
  }
  if(!nh.param<std::string>("steer_joint_name", g_steer_joint_name, "steer_joint"))
  {
    ROS_WARN("failed to get parameter of steer_joint_name, use %s as default value.", "steer_joint");
  }
  if(!nh.param<std::string>("rear_drive_joint_name", g_rear_drive_joint_name, "rear_drive_joint"))
  {
    ROS_WARN("failed to get parameter of rear_drive_joint_name, use %s as default value.", "rear_drive_joint");
  }
  
  ros::Rate r(publish_rate);
  
  g_rear_drive_joint_velocity = 0;// for safety
  ros::spinOnce();// spin once to get inital values of global variables
  
  double eq_vel_steer2drive;// equivalent velocity from steering to driving for left_front_wheel (yeild when robot doesn't move)
  std_msgs::Float64 angular_velocity_to_pub;
  while(ros::ok())
  {
    // calculate the velocity to publish
    eq_vel_steer2drive = -g_steer_joint_velocity*wheel_seperation_w/2/wheel_radius;
    angular_velocity_to_pub.data = g_rear_drive_joint_velocity/wheel_radius*cos(g_steer_joint_position) + eq_vel_steer2drive;
    
    // publish velocity command for front left wheel
    velocity_pub.publish(angular_velocity_to_pub);
    
    ros::spinOnce();// todo: subscribe cmd and joint_state at different rate
    r.sleep();
  }
  
  return 0;
}
