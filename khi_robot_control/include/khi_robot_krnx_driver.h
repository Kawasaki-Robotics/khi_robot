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

#ifndef KHI_ROBOT_KRNX_DRIVER_H
#define KHI_ROBOT_KRNX_DRIVER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <khi_robot_driver.h>
#include <krnx.h>

namespace khi_robot_control
{
#define KRNX_MSGSIZE 1024
#define KRNX_STDAXES 6
#define KRNX_MOTION_BUF 10
#define KRNX_PRINT_TH 1000

enum KrnxJointType
{
    TYPE_RAD,
    TYPE_LINE
};

struct KrnxJointTable
{
    float home;
    int type;
    std::string joint_name;
};
struct KrnxArmTable
{
    int jt_num;
    KrnxJointTable jt_tbl[KHI_MAX_JOINT];
};
struct KrnxRobotTable
{
    std::string robot_name;
    int arm_num;
    KrnxArmTable arm_tbl[KRNX_MAX_ROBOT];
};

class KhiRobotKrnxDriver : public KhiRobotDriver
{
public:
    KhiRobotKrnxDriver();
    ~KhiRobotKrnxDriver();

    bool initialize( const int cont_no, const std::string robot_name, const double period, const JointData joint, bool in_simulation = false );
    bool open( const int cont_no, const std::string ip_address );
    bool close( const int cont_no );
    bool activate( const int cont_no, JointData *joint );
    bool deactivate( const int cont_no );
    bool readData( const int cont_no, JointData *joint );
    bool writeData( const int cont_no, JointData joint );
    bool updateState( const int cont_no );
    bool getPeriodDiff( const int cont_no, double *diff );
    bool commandHandler( khi_robot_msgs::KhiRobotCmd::Request &req, khi_robot_msgs::KhiRobotCmd::Response &res );

private:
    /* general */
    char msg_buf[KRNX_MSGSIZE];
    bool do_restart[KRNX_MAX_CONTROLLER];
    bool do_quit[KRNX_MAX_CONTROLLER];

    /* RTC */
    float rtc_comp[KRNX_MAX_CONTROLLER][KRNX_MAX_ROBOT][KRNX_MAXAXES];
    float rtc_old_comp[KRNX_MAX_CONTROLLER][KRNX_MAX_ROBOT][KRNX_MAXAXES];
    int rtc_status[KRNX_MAX_CONTROLLER][KRNX_MAX_ROBOT][KRNX_MAXAXES];
    int rtc_seq_no[KRNX_MAX_CONTROLLER];
    KrnxRobotTable *p_rb_tbl[KRNX_MAX_CONTROLLER];

    bool getCurMotionData( const int cont_no, const int robot_no, TKrnxCurMotionData *p_motion_data = NULL );
    bool retKrnxRes( const int cont_no, const std::string name, const int ret, bool error = true );
    bool conditionCheck( const int cont_no );
    bool setJointDataHome( const int cont_no, JointData *joint );
    std::vector<std::string> splitString( const std::string str, const char del );
};

} // namespace

#endif // KHI_ROBOT_KRNX_DRIVER_H