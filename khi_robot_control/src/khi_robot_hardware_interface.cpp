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
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <urdf/model.h>

namespace khi_robot_control
{
KhiRobotHardwareInterface::KhiRobotHardwareInterface()
{
}

KhiRobotHardwareInterface::~KhiRobotHardwareInterface()
{
    deactivate();
    close();
}

bool KhiRobotHardwareInterface::open( const std::string& robot_name, const std::string& ip_address, const double& period, const bool in_simulation )
{
    ros::NodeHandle nh_joints;
    std::vector<std::string> controller_names, joint_names;
    std::stringstream param[KHI_MAX_ARM];
    int jt;

    std::string ns = ros::this_node::getNamespace();
    urdf::Model model;
    model.initParam( ns + "/robot_description" );
    data.robot_name = robot_name;

    data.arm_num = 0;
    for ( int ano = 0; ano < KHI_MAX_ARM; ano++ )
    {
        jt = 0;
        data.arm[ano].jt_num = 0;
        param[ano] << "khi_robot_param/arm/arm" << ano + 1;
        if ( nh_joints.getParam( param[ano].str(), controller_names ) )
        {
            data.arm_num++;
            for ( int cno = 0; cno < controller_names.size(); cno++ )
            {
                if ( nh_joints.getParam( controller_names[cno] + "/joints", joint_names ) )
                {
                    for ( int n = 0; n < joint_names.size(); n++ )
                    {
                        data.arm[ano].name[jt] = joint_names[n];
                        auto jt_ptr = model.getJoint( joint_names[n] );
                        data.arm[ano].type[jt] = jt_ptr->type;
                        hardware_interface::JointStateHandle state_handle( data.arm[ano].name[jt], &data.arm[ano].pos[jt], &data.arm[ano].vel[jt], &data.arm[ano].eff[jt] );
                        joint_state_interface.registerHandle( state_handle );
                        hardware_interface::JointHandle pos_handle( joint_state_interface.getHandle( data.arm[ano].name[jt] ), &data.arm[ano].cmd[jt] );
                        joint_position_interface.registerHandle( pos_handle );
                        ros::NodeHandle nh_limits(robot_name);
                        joint_limits_interface::JointLimits limits;

                        joint_limits_interface::getJointLimits( data.arm[ano].name[jt], nh_limits, limits );
                        joint_limits_interface::PositionJointSaturationHandle limits_handle( joint_position_interface.getHandle( data.arm[ano].name[jt] ), limits );
                        joint_limit_interface.registerHandle( limits_handle );
                        jt++;
                    }
                }
                else
                {
                    ROS_ERROR( "Failed to get param '/joints'" );
                    return false;
                }
            }
             data.arm[ano].jt_num = jt;
        }
    }

    if ( in_simulation )
    {
        ROS_INFO_STREAM_NAMED( "khi_robot","KHI Robot Hardware Interface in simulation mode" );
    }

    registerInterface( &joint_state_interface );
    registerInterface( &joint_position_interface );

    /* start KhiRobotClient */
    client = new KhiRobotClient();
    return client->open( ip_address, period, data, in_simulation );
}

bool KhiRobotHardwareInterface::activate()
{
    joint_limit_interface.reset();
    return client->activate( data );
}

bool KhiRobotHardwareInterface::hold()
{
    return client->hold( data );
}

void KhiRobotHardwareInterface::deactivate()
{
    client->deactivate( data );
}

void KhiRobotHardwareInterface::close()
{
    client->close();
    delete client;
}

void KhiRobotHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    client->read( data );
}

void KhiRobotHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    joint_limit_interface.enforceLimits( period );
    client->write( data );
}

int KhiRobotHardwareInterface::updateState()
{
    return client->updateState( data );
}

int KhiRobotHardwareInterface::getStateTrigger()
{
    return client->getStateTrigger();
}

bool KhiRobotHardwareInterface::getPeriodDiff( double& diff )
{
    return client->getPeriodDiff( diff );
}
} // namespace
