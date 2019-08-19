/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
// #include <random>

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_, flag_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher flag_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool flag_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(1),
  angular_(0),
  deadman_axis_(4),
  flag_axis_(3),
  l_scale_(0.3),
  a_scale_(0.9)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);


  std::cout<< "hello" << "\n";
  deadman_pressed_ = false;
  zero_twist_published_ = false;
  flag_pressed_=false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  flag_pub_ = nh_.advertise<geometry_msgs::PointStamped>("flag", 1, true);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);
  
    // std::random_device dev;
    // std::mt19937 rng(dev());
    // std::uniform_int_distribution<std::mt19937::result_type> dist6(1,6);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  flag_pressed_=joy->buttons[flag_axis_];
  ROS_INFO("show the pressing button");
  std::cout<< deadman_axis_<< "\n";
  //x button
  std::cout<< flag_axis_<< "\n";

  std::cout<< "status of the joy "<<flag_pressed_<< "\n";
  
}

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if(flag_pressed_)
  {
    geometry_msgs::PointStamped point;
    point.header.frame_id="point";
    point.header.seq=0;
    point.header.stamp=ros::Time::now();
    point.point.x=10;  
    point.point.y=10;
    point.point.z=0;
    flag_pub_.publish(point);
    //  in case you not release the key and it keeps publishing
    flag_pressed_=false;
  }
  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "z");
  TurtlebotTeleop turtlebot_teleop;

  ros::spin();
}
