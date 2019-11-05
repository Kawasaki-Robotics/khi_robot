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

#ifndef KHI_ROBOT_DRIVER_H
#define KHI_ROBOT_DRIVER_H

#include <string>
#include <khi_robot_msgs/KhiRobotCmd.h>

namespace khi_robot_control
{
#define KHI_MAX_CONTROLLER 8
#define KHI_MAX_JOINT 18
#define KHI_MAX_SIG_SIZE 512

struct JointData
{
    int joint_num;
    std::string name[KHI_MAX_JOINT];
    double cmd[KHI_MAX_JOINT];
    double pos[KHI_MAX_JOINT];
    double vel[KHI_MAX_JOINT];
    double eff[KHI_MAX_JOINT];
};

struct RobotInfo
{
    int state;
    int state_trigger;
    std::string ip_address;
    std::string robot_name;
    int arm_num;
    double period;
};

enum KhiRobotState
{
    STATE_MIN = -1,
    INIT,
    CONNECTING,
    INACTIVE,
    ACTIVATING,
    ACTIVE,
    HOLDED,
    DEACTIVATING,
    DISCONNECTING,
    DISCONNECTED,
    ERROR,
    NOT_REGISTERED,
    STATE_MAX
};
const static std::string KhiRobotStateName[STATE_MAX] =
{
    "INIT",
    "CONNECTING",
    "INACTIVE",
    "ACTIVATING",
    "ACTIVE",
    "HOLDED",
    "DEACTIVATING",
    "DISCONNECTING",
    "DISCONNECTED",
    "ERROR",
    "NOT_REGISTERED"
};

enum KhiRobotStateTrigger
{
    TRIGGER_MIN = -1,
    NONE,
    HOLD,
    RESTART,
    QUIT,
    TRIGGER_MAX
};
const static std::string KhiRobotStateTriggerName[TRIGGER_MAX] =
{
    "NONE",
    "HOLD",
    "RESTART",
    "QUIT"
};

class KhiRobotDriver
{
public:
    KhiRobotDriver()
    {
        for ( int cno = 0; cno < KHI_MAX_CONTROLLER; cno++ )
        {
            robot_info[cno].state = INIT;
            robot_info[cno].state_trigger = NONE;
            robot_info[cno].ip_address = "127.0.0.1";
            robot_info[cno].robot_name = "";
            robot_info[cno].arm_num = -1;
        }

        driver_name = __func__;
    }

    int getState( const int cont_no )
    {
        if ( ( cont_no < 0 ) || ( cont_no > KHI_MAX_CONTROLLER ) ) { return NOT_REGISTERED; }
        else                                                       { return robot_info[cont_no].state; }
    }

    std::string getStateName( const int cont_no )
    {
        int state;
        std::string name = "";

        state = getState( cont_no );
        if ( ( state > STATE_MIN ) && ( state < STATE_MAX ) )
        {
            name = KhiRobotStateName[state];
        }

        return name;
    }

    bool setState( const int cont_no, const int state )
    {
        if ( !contLimitCheck( cont_no, KHI_MAX_CONTROLLER ) ) { return false; }

        if ( ( state <= STATE_MIN ) || ( state >= STATE_MAX ) )
        {
            return false;
        }
        else
        {
            if ( robot_info[cont_no].state != state )
            {
                infoPrint( "State %d: %s -> %s", cont_no, KhiRobotStateName[robot_info[cont_no].state].c_str(), KhiRobotStateName[state].c_str() );
                robot_info[cont_no].state = state;
            }
            return true;
        }
    }

    bool isTransitionState( const int cont_no )
    {
        int state;

        state = getState( cont_no );
        if ( ( state == CONNECTING ) || ( state == ACTIVATING ) || ( state == DEACTIVATING ) || ( state == DISCONNECTING ) )
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int getStateTrigger( const int cont_no )
    {
        int state_trigger;

        if ( ( cont_no < 0 ) || ( cont_no > KHI_MAX_CONTROLLER ) ) { return NONE; }
        else
        {
            state_trigger = robot_info[cont_no].state_trigger;
            robot_info[cont_no].state_trigger = NONE;
            return state_trigger;
        }
    }

    bool setStateTrigger( const int cont_no, const int state_trigger )
    {
        if ( !contLimitCheck( cont_no, KHI_MAX_CONTROLLER ) ) { return false; }

        if ( ( state_trigger <= TRIGGER_MIN ) || ( state_trigger >= TRIGGER_MAX ) )
        {
            return false;
        }
        else
        {
            if ( robot_info[cont_no].state_trigger != NONE )
            {
                warnPrint( "State Trigger is already done %d: %s", cont_no, KhiRobotStateTriggerName[state_trigger].c_str() );
            }
            infoPrint( "State Trigger %d: %s", cont_no, KhiRobotStateTriggerName[state_trigger].c_str() );
            robot_info[cont_no].state_trigger = state_trigger;
            return true;
        }
    }

    bool setRobotName( const int cont_no, const std::string name )
    {
        if ( contLimitCheck( cont_no, KHI_MAX_CONTROLLER ) ) { return false; }

        robot_info[cont_no].robot_name = name;
        return true;
    }

    bool contLimitCheck( const int cont_no, const int limit )
    {
        if ( ( cont_no < 0 ) || ( cont_no > KHI_MAX_CONTROLLER ) || ( cont_no > limit ) )
        {
            errorPrint( "contLimitCheck ERROR!" );
            return false;
        }
        else
        {
            return true;
        }
    }

    void infoPrint( const char *format, ... )
    {
        char msg[512] = { 0 };
        va_list ap;

        va_start( ap, format );
        vsnprintf( msg, sizeof(msg), format, ap );
        va_end( ap );
        ROS_INFO( "[%s] %s", driver_name.c_str(), msg );
    }

    void warnPrint( const char *format, ... )
    {
        char msg[512] = { 0 };
        va_list ap;

        va_start( ap, format );
        vsnprintf( msg, sizeof(msg), format, ap );
        va_end( ap );
        ROS_WARN( "[%s] %s", driver_name.c_str(), msg );
    }

    void errorPrint( const char *format, ... )
    {
        char msg[512] = { 0 };
        va_list ap;

        va_start( ap, format );
        vsnprintf( msg, sizeof(msg), format, ap );
        va_end( ap );
        ROS_ERROR( "[%s] %s", driver_name.c_str(), msg );
    }

    void jointPrint( std::string name, const JointData joint )
    {
        char msg[512] = { 0 };
        char jt_val[16] = { 0 };

        snprintf( msg, sizeof(msg), "[%s]\t", name.c_str() );
        if ( name == std::string("write") )
        {
            for ( int cnt = 0; cnt < joint.joint_num; cnt++ )
            {
                snprintf( jt_val, sizeof(jt_val), "%.3lf\t", joint.cmd[cnt] );
                strcat( msg, jt_val );
            }
        }
        else
        {
            for ( int cnt = 0; cnt < joint.joint_num; cnt++ )
            {
                snprintf( jt_val, sizeof(jt_val), "%.3lf\t", joint.pos[cnt] );
                strcat( msg, jt_val );
            }
        }
        infoPrint( "[SIM]%s", msg );
    }

    virtual ~KhiRobotDriver() {};
    virtual bool initialize( const int cont_no, const std::string robot_name, const double period, const JointData joint, bool in_simulation = false ) = 0;
    virtual bool open( const int cont_no, const std::string ip_address ) = 0;
    virtual bool close( const int cont_no ) = 0;
    virtual bool activate( const int cont_no, JointData *joint ) = 0;
    virtual bool hold( const int cont_no, const JointData joint ) = 0;
    virtual bool deactivate( const int cont_no ) = 0;
    virtual bool readData( const int cont_no, JointData *joint ) = 0;
    virtual bool writeData( const int cont_no, JointData joint ) = 0;
    virtual bool updateState( const int cont_no ) = 0;
    virtual bool getPeriodDiff( const int cont_no, double *diff ) = 0;
    virtual bool commandHandler( khi_robot_msgs::KhiRobotCmd::Request &req, khi_robot_msgs::KhiRobotCmd::Response &res ) = 0;

protected:
    bool in_simulation;
    std::string driver_name;
    RobotInfo robot_info[KHI_MAX_CONTROLLER];
    int return_code;
    int error_code;
};

} // namespace

#endif // KHI_ROBOT_DRIVER_H