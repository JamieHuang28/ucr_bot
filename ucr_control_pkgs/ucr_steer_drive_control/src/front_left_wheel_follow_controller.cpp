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
#include "std_msgs/Float64.h"

std_msgs::Float64 g_velocity_to_pub;

void CommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  g_velocity_to_pub.data = msg->linear.x;
  ROS_DEBUG_STREAM("CommandCallback is called, and arg = " << *msg << " , result = " << g_velocity_to_pub);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_left_wheel_follow_controller");
  
  ros::NodeHandle nh("~");
  
  ros::Publisher velocity_pub = nh.advertise<std_msgs::Float64>("velocity", 1);
  ros::Subscriber cmd_sub = nh.subscribe("/steer_drive_controller/cmd_vel", 1, CommandCallback);
  
  int publish_rate;
  double wheel_seperation_h;// between front wheel and rear wheel
  double wheel_seperation_w;// between left wheel and right wheel
  double wheel_radius;
  
  // get parameters
  if(!nh.param<int>("publish_rate", publish_rate, 50))
  {
    ROS_WARN("failed to get parameter of publish_rate, use %d as default value.", 50);
  }
  if(!nh.param<double>("wheel_seperation_h", wheel_seperation_h, 1.0))
  {
    ROS_WARN("failed to get parameter of wheel_seperation_h, use %.2f as default value.", 1.0);
  }
  if(!nh.param<double>("wheel_seperation_w", wheel_seperation_w, 1.0))
  {
    ROS_WARN("failed to get parameter of wheel_seperation_w, use %.2f as default value.", 1.0);
  }
  if(!nh.param<double>("wheel_radius", wheel_radius, 0.5))
  {
    ROS_WARN("failed to get parameter of wheel_radius, use %.2f as default value.", 0.5);
  }
  
  ros::Rate r(publish_rate);
  
  g_velocity_to_pub.data = 0;
  ros::spinOnce();// spin once to get first command
  
  while(ros::ok())
  {
    // calculate the velocity to publish
    
    // publish velocity command for front left wheel
    velocity_pub.publish(g_velocity_to_pub);
    
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
