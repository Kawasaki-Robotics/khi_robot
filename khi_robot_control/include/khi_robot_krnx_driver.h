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

#include <mutex>
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

struct KhiRobotKrnxRtcData
{
    int sw;
    int seq_no;
    float comp[KRNX_MAX_ROBOT][KRNX_MAXAXES];
    float old_comp[KRNX_MAX_ROBOT][KRNX_MAXAXES];
    int status[KRNX_MAX_ROBOT][KRNX_MAXAXES];
};

class KhiRobotKrnxDriver : public KhiRobotDriver
{
public:
    KhiRobotKrnxDriver();
    ~KhiRobotKrnxDriver();
    bool setState( const int& cont_no, const int& state );

    bool initialize( const int& cont_no, const double& period, KhiRobotData& data, const bool in_simulation = false ) override;
    bool open( const int& cont_no, const std::string& ip_address, KhiRobotData& data ) override;
    bool close( const int& cont_no ) override;
    bool activate( const int& cont_no, KhiRobotData& data ) override;
    bool hold( const int& cont_no, const KhiRobotData& data ) override;
    bool deactivate( const int& cont_no, const KhiRobotData& data ) override;
    bool readData( const int& cont_no, KhiRobotData& data ) override;
    bool writeData( const int& cont_no, const KhiRobotData& data ) override;
    bool updateState( const int& cont_no, const KhiRobotData& data ) override;
    bool getPeriodDiff( const int& cont_no, double& diff ) override;
    bool commandHandler( khi_robot_msgs::KhiRobotCmd::Request& req, khi_robot_msgs::KhiRobotCmd::Response& res ) override;

private:
    /* general */
    char cmd_buf[KRNX_MSGSIZE];
    char msg_buf[KRNX_MSGSIZE];
    int sw_dblrefflt[KRNX_MAX_CONTROLLER];
    std::mutex mutex_state[KRNX_MAX_CONTROLLER];

    /* RTC */
    KhiRobotKrnxRtcData rtc_data[KRNX_MAX_CONTROLLER];

    bool getCurMotionData( const int& cont_no, const int& robot_no, TKrnxCurMotionData* p_motion_data );
    int execAsMonCmd( const int& cont_no, const char* cmd, char* buffer, int buffer_sz, int* as_err_code );
    bool retKrnxRes( const int& cont_no, const std::string& name, const int& ret, const bool error = true );
    bool conditionCheck( const int& cont_no, const KhiRobotData& data );
    bool setRobotDataHome( const int& cont_no, KhiRobotData& data );
    std::vector<std::string> splitString( const std::string& str, const char& del );
    bool loadDriverParam( const int& cont_no, KhiRobotData& data );
    bool loadRtcProg( const int& cont_no, const std::string& name );
    bool syncRtcPos( const int& cont_no, KhiRobotData& data );
};

} // namespace

#endif // KHI_ROBOT_KRNX_DRIVER_H