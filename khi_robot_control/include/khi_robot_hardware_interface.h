/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Kawasaki Heavy Industries, LTD.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef KHI_ROBOT_HARDWARE_INTERFACE_
#define KHI_ROBOT_HARDWARE_INTERFACE_

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <khi_robot_client.h>

namespace khi_robot_control
{

class KhiRobotHardwareInterface : public hardware_interface::RobotHW
{
public:
    KhiRobotHardwareInterface();
    ~KhiRobotHardwareInterface();

    bool open( std::string robot_name, std::string ip_address, double period, bool in_simulation = false );
    bool activate();
    bool hold();
    void deactivate();
    void close();
    void read( const ros::Time time, const ros::Duration period );
    void write( const ros::Time time, const ros::Duration period );
    int updateState();
    int getStateTrigger();
    bool getPeriodDiff( double *diff );

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_position_interface;
    joint_limits_interface::PositionJointSaturationInterface joint_limit_interface;

    khi_robot_control::JointData joint;
    khi_robot_control::KhiRobotClient *client;
};

} // namespace

#endif // KHI_ROBOT_HARDWARE_INTERFACE_
