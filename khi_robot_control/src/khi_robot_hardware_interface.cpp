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

#include <khi_robot_hardware_interface.h>

namespace khi_robot_control
{
KhiRobotHardwareInterface::KhiRobotHardwareInterface()
{
}

KhiRobotHardwareInterface::~KhiRobotHardwareInterface()
{
    deactivate();
}

bool KhiRobotHardwareInterface::open( std::string robot_name, std::string ip_address, double period, bool in_simulation )
{
    ros::NodeHandle nh;
    std::vector<std::string> controller_names, joint_names;
    bool is_success = false;
    int cnt = 0;

    joint.joint_num = 0;
    if ( nh.getParam( "khi_robot_controllers/names", controller_names ) )
    {
        for ( int cno = 0; cno < controller_names.size(); cno++ )
        {
            if ( nh.getParam( controller_names[cno] + "/joints", joint_names ) )
            {
                for ( int jno = 0; jno < joint_names.size(); jno++ )
                {
                    joint.name[cnt] = joint_names[jno];
                    hardware_interface::JointStateHandle state_handle( joint.name[cnt], &joint.pos[cnt], &joint.vel[cnt], &joint.eff[cnt] );
                    joint_state_interface.registerHandle( state_handle );
                    hardware_interface::JointHandle pos_handle( joint_state_interface.getHandle( joint.name[cnt] ), &joint.cmd[cnt] );
                    joint_position_interface.registerHandle( pos_handle );
                    cnt++;
                }
                joint.joint_num += joint_names.size();
            }
            else
            {
                ROS_ERROR( "Failed to get param '/joints'" );
                return false;
            }
        }
    }
    else
    {
        ROS_ERROR( "Failed to get param 'khi_robot_controllers/names'" );
        return false;
    }

    if ( in_simulation )
    {
        ROS_INFO_STREAM_NAMED("khi_robot","KHI Robot Hardware Interface in simulation mode");
    }

    registerInterface( &joint_state_interface );
    registerInterface( &joint_position_interface );

    /* start KhiRobotClient */
    client = new KhiRobotClient();
    return client->open( robot_name, ip_address, period, joint, in_simulation );
}

bool KhiRobotHardwareInterface::activate()
{
    return client->activate( &joint );
}

void KhiRobotHardwareInterface::deactivate()
{
    client->deactivate();
    delete client;
}

void KhiRobotHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
    client->read( &joint );
}

void KhiRobotHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
    client->write( joint );
}

int KhiRobotHardwareInterface::getState()
{
    return client->getState();
}

bool KhiRobotHardwareInterface::getPeriodDiff( double *diff )
{
    return client->getPeriodDiff( diff );
}
} // namespace
