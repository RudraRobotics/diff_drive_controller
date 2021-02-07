/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <medicbot_control/medicbot_hw_interface.h>
#include <exception>
#include <algorithm>
#include <iostream>

#include <bits/stdc++.h> 
#include <boost/algorithm/string.hpp>

namespace rrbot_control
{

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh_, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh_, urdf_model)
{
    ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");

    nhp1_ = ros::NodeHandle("~");
    nhp2_ = ros::NodeHandle("~");
    
    if (!nhp1_.hasParam("my_pid1/p")) 
        nhp1_.setParam("my_pid1/p", 100.0);
    if (!nhp1_.hasParam("my_pid1/i"))
        nhp1_.setParam("my_pid1/i", 10.0);
    if (!nhp1_.hasParam("my_pid1/d"))
        nhp1_.setParam("my_pid1/d", 0.0);
    if (!nhp1_.hasParam("my_pid1/i_clamp_min"))
        nhp1_.setParam("my_pid1/i_clamp_min", -10.0);
    if (!nhp1_.hasParam("my_pid1/i_clamp_max"))
        nhp1_.setParam("my_pid1/i_clamp_max", 10.0);
    
    if (!nhp2_.hasParam("my_pid2/p")) 
        nhp2_.setParam("my_pid2/p", 100.0);
    if (!nhp2_.hasParam("my_pid2/i"))
        nhp2_.setParam("my_pid2/i", 10.0);
    if (!nhp2_.hasParam("my_pid2/d"))
        nhp2_.setParam("my_pid2/d", 0.0);
    if (!nhp2_.hasParam("my_pid2/i_clamp_min"))
        nhp2_.setParam("my_pid2/i_clamp_min", -10.0);
    if (!nhp2_.hasParam("my_pid2/i_clamp_max"))
        nhp2_.setParam("my_pid2/i_clamp_max", 10.0);
    
    nhp1_.setParam("publish_state", true);
    nhp2_.setParam("publish_state", true);
    
    my_pid1_.init(ros::NodeHandle(nhp1_, "my_pid1"), false);
    my_pid2_.init(ros::NodeHandle(nhp2_, "my_pid2"), false);

    min_ang = 0.00315167802;
    prev_right_wheel_pos = 0;
    prev_left_wheel_pos = 0;
    Setup();
}

RRBotHWInterface::~RRBotHWInterface() {
    Write("0,0,,,,,,,,,,,,,");
    ros::Duration(0.01).sleep();
}

void RRBotHWInterface::read(ros::Duration &elapsed_time)
{
    const char* enc_data = Read();  
    std::vector<std::string> result;  
    boost::split(result, enc_data, boost::is_any_of(","));
    if(result.size()==2) {
    	    int curr_left_wheel_pos = atof(result[0].c_str());
	    int curr_right_wheel_pos = atof(result[1].c_str());
	    int delta_left_wheel_pos =  curr_left_wheel_pos - prev_left_wheel_pos;
	    int delta_right_wheel_pos = curr_right_wheel_pos - prev_right_wheel_pos;
	    double dt = elapsed_time.toSec();
	    joint_position_[0] += delta_left_wheel_pos * min_ang; 
	    joint_velocity_[0] = (delta_left_wheel_pos * min_ang)/dt;
	    joint_position_[1] += delta_right_wheel_pos * min_ang; 
	    joint_velocity_[1] = (delta_right_wheel_pos * min_ang)/dt;
	    // std::cout<<"feedback="<<joint_velocity_[0]<<"\t"<<joint_velocity_[1]<<std::endl;
	    prev_left_wheel_pos = curr_left_wheel_pos;
	    prev_right_wheel_pos = curr_right_wheel_pos;

    }
}

void RRBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);
  tnow = ros::Time::now();
  ros::Duration dt = tnow - last_cmd_time_;
  double error1 = joint_velocity_command_[0] - joint_velocity_[0];
  double error2 = joint_velocity_command_[1] - joint_velocity_[1];
  // std::cout<<"cmd="<<joint_velocity_command_[0]<<"\t"<<joint_velocity_command_[1]<<std::endl;
  double joint1_out_pow = my_pid1_.computeCommand(error1, dt);
  double joint2_out_pow = my_pid2_.computeCommand(error2, dt);
  int pwm1 = static_cast<int>(joint1_out_pow);
  int pwm2 = static_cast<int>(joint2_out_pow);
  if(pwm1>255)
	  pwm1 = 255;
  else if(pwm1<-255)
	  pwm1=-255;
  if(pwm2>255)
	  pwm2=255;
  else if(pwm2<-255)
	  pwm2=-255;
  std::string pwm_datas = std::to_string(pwm1) + "," + std::to_string(pwm2);
  while(pwm_datas.length()<16) {
    pwm_datas.push_back(',');
  }
  const char *res = pwm_datas.c_str(); 
  if(pwm_datas.length()==16) {
    Write(res); //std::cout<<"pwm="<<res<<std::endl; 
    last_cmd_time_ = tnow;
    ros::Duration(0.01).sleep();
  }
}

void RRBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace

