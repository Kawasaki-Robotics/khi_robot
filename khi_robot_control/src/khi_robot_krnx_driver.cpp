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

#include <khi_robot_krnx_driver.h>

namespace khi_robot_control
{
#define KHI_ROBOT_WD002N "WD002N"
#define KHI_ROBOT_RS007L "RS007L"
#define KHI_ROBOT_RS007N "RS007N"
#define KHI_ROBOT_RS080N "RS080N"
#define KHI_KRNX_BUFFER_SIZE 4
#define KHI_KRNX_ACTIVATE_TH 0.02
#define KHI_KRNX_M2MM 1000

const static KrnxArmTable DOF4L_ARM_TBL =
{
    4, { { -45.0f*M_PI/180, TYPE_RAD, "lower_joint1" }, { 45.0f*M_PI/180, TYPE_RAD, "lower_joint2" }, { 90.0f/KHI_KRNX_M2MM, TYPE_LINE, "lower_joint3" }, { 0.0f, TYPE_RAD, "lower_joint4" }, { 0 } }
};
const static KrnxArmTable DOF4U_ARM_TBL =
{
    4, { { 45.0f*M_PI/180, TYPE_RAD, "upper_joint1" }, { -45.0f*M_PI/180, TYPE_RAD, "upper_joint2" }, { 90.0f/KHI_KRNX_M2MM, TYPE_LINE, "upper_joint3" }, { 0.0f, TYPE_RAD, "upper_joint4" }, { 0 } }
};
const static KrnxArmTable DOF6_ARM_TBL =
{
    6, { { 0.0f, TYPE_RAD, "joint1" }, { 0.0f, TYPE_RAD, "joint2" }, { 0.0f, TYPE_RAD, "joint3" }, { 0.0f, TYPE_RAD, "joint4" }, { 0.0f, TYPE_RAD, "joint5" }, { 0.0f, TYPE_RAD, "joint6" }, { 0 } }
};

static KrnxRobotTable rb_tbl[5] =
{
    { KHI_ROBOT_WD002N, 2, { DOF4L_ARM_TBL, DOF4U_ARM_TBL, { 0 } } },
    { KHI_ROBOT_RS007N, 1, { DOF6_ARM_TBL, { 0 } } },
    { KHI_ROBOT_RS007L, 1, { DOF6_ARM_TBL, { 0 } } },
    { KHI_ROBOT_RS080N, 1, { DOF6_ARM_TBL, { 0 } } },
    { "LAST", -1, { 0 } }
};

KhiRobotKrnxDriver::KhiRobotKrnxDriver() : KhiRobotDriver()
{
    driver_name = __func__;
    for ( int cno = 0; cno < KRNX_MAX_CONTROLLER; cno++ )
    {
        rtc_seq_no[cno]= 0;
        sw_dblrefflt[cno] = 0;
        now_as_mon_cmd[cno] = false;
    }
}

KhiRobotKrnxDriver::~KhiRobotKrnxDriver()
{
    int state;

    for ( int cno = 0; cno < KRNX_MAX_CONTROLLER; cno++ )
    {
        state = getState( cno );
        if ( ( state != INIT ) && ( state != DISCONNECTED ) )
        {
            infoPrint("destructor");
            deactivate( cno );
            close( cno );
        }
    }
}

bool KhiRobotKrnxDriver::setState( const int cont_no, const int state )
{
    while( now_as_mon_cmd[cont_no] )
    {
        ros::Duration(robot_info[cont_no].period/1e+9).sleep();
    }

    KhiRobotDriver::setState( cont_no, state );
}

int KhiRobotKrnxDriver::execAsMonCmd( const int cont_no, const char *cmd, char *buffer, int buffer_sz, int *as_err_code )
{
    now_as_mon_cmd[cont_no] = true;

    return_code = krnx_ExecMon( cont_no, cmd, buffer, buffer_sz, as_err_code );
    retKrnxRes( cont_no, "krnx_ExecMon()", return_code );

    now_as_mon_cmd[cont_no] = false;

    return return_code;
}

bool KhiRobotKrnxDriver::retKrnxRes( const int cont_no, const std::string name, const int ret, bool error )
{
    if ( ret != KRNX_NOERROR )
    {
        errorPrint( "%s returned -0x%X", name.c_str(), -ret );
        if ( error ) { setState( cont_no, ERROR ); }
        return false;
    }
    else
    {
        return true;
    }
}

/* This function needs some communication time. Don't use this in control loop */
bool KhiRobotKrnxDriver::conditionCheck( const int cont_no )
{
    TKrnxPanelInfo panel_info;
    bool ret = true;

    if ( getState( cont_no ) == ERROR )
    {
        return false;
    }

    if ( in_simulation ) { return true; }

    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        /* Condition Check */
        return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
        if ( !retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) )
        {
            ret = false;
        }

        if ( panel_info.repeat_lamp != -1 )
        {
            errorPrint( "Please change Robot Controller's TEACH/REPEAT to REPEAT" );
            ret = false;
        }
        if ( panel_info.teach_lock_lamp != 0 )
        {
            errorPrint( "Please change Robot Controller's TEACH LOCK to OFF" );
            ret = false;
        }
        else if ( panel_info.run_lamp != -1 )
        {
            errorPrint( "Please change Robot Controller's RUN/HOLD to RUN" );
            ret = false;
        }
        else if ( panel_info.emergency != 0 )
        {
            errorPrint( "Please change Robot Controller's EMERGENCY to OFF" );
            ret = false;
        }
    }

    if ( !ret )
    {
        setState( cont_no, ERROR );
    }

    return ret;
}

bool KhiRobotKrnxDriver::initialize( const int cont_no, const std::string robot_name, const double period, const JointData joint, bool in_simulation )
{
    char msg[256] = { 0 };

    // robot info
    robot_info[cont_no].robot_name = robot_name;
    robot_info[cont_no].period = period;
    for ( int idx = 0; rb_tbl[idx].arm_num != -1; idx++ )
    {
        if ( robot_info[cont_no].robot_name == rb_tbl[idx].robot_name )
        {
            robot_info[cont_no].arm_num = rb_tbl[idx].arm_num;

            // rb_tbl
            p_rb_tbl[cont_no] = &rb_tbl[idx];
            break;
        }
    }

    return_code = krnx_GetKrnxVersion( msg, sizeof(msg) );
    infoPrint( msg );

    this->in_simulation = in_simulation;

    return true;
}

bool KhiRobotKrnxDriver::open( const int cont_no, const std::string ip_address )
{
    char c_ip_address[64] = { 0 };

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( getState( cont_no ) != INIT )
    {
        warnPrint( "Cannot open cont_no:%d because it is already opend...", cont_no );
        return false;
    }

    if ( in_simulation )
    {
        setState( cont_no, CONNECTING );
        setState( cont_no, INACTIVE );
        return true;
    }


    setState( cont_no, CONNECTING );
    strncpy( c_ip_address, ip_address.c_str(), sizeof(c_ip_address) );
    infoPrint( "Connecting to real controller: %s", c_ip_address );
    return_code = krnx_Open( cont_no, c_ip_address );
    if ( return_code == cont_no )
    {
        robot_info[cont_no].ip_address = ip_address;
        if ( !loadDriverParam( cont_no ) ) { return false; };

        setState( cont_no, INACTIVE );
        return true;
    }
    else
    {
        retKrnxRes( cont_no, "krnx_Open", return_code, false );
        setState( cont_no, INIT );
        return false;
    }
}

bool KhiRobotKrnxDriver::close( const int cont_no )
{
    char msg[1024] = { 0 };

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( in_simulation )
    {
        setState( cont_no, DISCONNECTING );
        setState( cont_no, DISCONNECTED );
        return true;
    }

    /* Switch Reversion */
    if ( sw_dblrefflt[cont_no] == -1 )
    {
        snprintf( msg, sizeof(msg), "SW ZDBLREFFLT_MODSTABLE=ON" );
        return_code = execAsMonCmd( cont_no, msg, msg_buf, sizeof(msg_buf), &error_code );
    }

    setState( cont_no, DISCONNECTING );
    return_code = krnx_Close( cont_no );
    if ( return_code == KRNX_NOERROR )
    {
        setState( cont_no, DISCONNECTED );
    }

    return retKrnxRes( cont_no, "krnx_Close", return_code, false );
}

bool KhiRobotKrnxDriver::activate( const int cont_no, JointData *joint )
{
    const int to_home_vel = 20; /* speed 20 */
    const float wait_time = 5.0f; /* 5 sec */
    const int timeout_cnt_th = wait_time/(robot_info[cont_no].period/1e+9);
    bool is_ready;
    TKrnxCurMotionData motion_data = { 0 };
    int rtc_sw = 0;
    int timeout_cnt = 0;
    int conv = 1;
    float diff = 0;
    FILE *fp;
    int error_lamp = 0;
    TKrnxPanelInfo panel_info;
    TKrnxProgramInfo program_info;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    setState( cont_no, ACTIVATING );
    if ( !conditionCheck( cont_no ) ) { return false; }

    if ( in_simulation )
    {
        setJointDataHome( cont_no, joint );
        setState( cont_no, ACTIVE );
        return true;
    }

    /* Preparation */
    bool is_cycle = true;
    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        return_code = krnx_GetCurErrorLamp( cont_no, ano, &error_lamp );
        if ( retKrnxRes( cont_no, "krnx_GetCurErrorLamp", return_code ) && ( error_lamp != 0 ) )
        {
            /* Error Reset */
            return_code = krnx_Ereset( cont_no, ano, &error_code );
        }
        return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
        if ( retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) )
        {
            if ( panel_info.cycle_lamp != -1 )
            {
                is_cycle = false;
            }
            if ( panel_info.motor_lamp != -1 )
            {
                /* Motor Power ON */
                return_code = execAsMonCmd( cont_no, "ZPOW ON", msg_buf, sizeof(msg_buf), &error_code );
                /* Error Reset */
                return_code = krnx_Ereset( cont_no, ano, &error_code );
            }
        }
       /* Clear RTC Comp Data */
        return_code = krnx_OldCompClear( cont_no, ano );
    }
    if ( is_cycle )
    {
        for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
        {
            return_code = krnx_GetProgramInfo( cont_no, ano, &program_info );
            if ( retKrnxRes( cont_no, "krnx_GetProgramInfo", return_code ) )
            {
                std::stringstream program;
                program << "rb_rtc" << ano + 1;
                if ( strcmp( program_info.robot[ano].program_name, program.str().c_str() ) == 0 )
                {
                    /* Already ROS Control */
                    setState( cont_no, ACTIVE );
                    return true;
                }
                else
                {
                    /* Not ROS Control */
                    return_code = krnx_Hold( cont_no, ano, &error_code );
                    ros::Duration(0.2).sleep();
                }
            }
        }
    }

    /* Sync RTC Position */
    if ( !syncRtcPos( cont_no ) )
    {
        errorPrint( "Failed to sync position" );
        setState( cont_no, ERROR );
        return false;
    }

    /* Set HOME position */
    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        /* Speed SLOW */
        return_code = krnx_SetMonSpeed( cont_no, ano, to_home_vel, &error_code );
        /* Executing base program */
        std::stringstream program;
        program << "rb_rtc" << ano + 1;
        return_code = krnx_Execute( cont_no, ano, program.str().c_str(), 1, 0, &error_code );

        while ( 1 )
        {
            ros::Duration(robot_info[cont_no].period/1e+9).sleep();
            timeout_cnt++;
            if ( timeout_cnt > timeout_cnt_th )
            {
                errorPrint( "Failed to activate: timeout" );
                setState( cont_no, ERROR );
                return false;
            }

            return_code = krnx_GetRtcSwitch( cont_no, ano, &rtc_sw );
            if ( ( return_code != KRNX_NOERROR ) || ( rtc_sw == 0 ) ) { continue; }

            return_code = krnx_GetCurMotionData( cont_no, ano, &motion_data );
            if ( return_code != KRNX_NOERROR ) { continue; }

            is_ready = true;
            for ( int jt = 0; jt < p_rb_tbl[cont_no]->arm_tbl[ano].jt_num; jt++ )
            {
                if ( p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].type == TYPE_LINE ) { conv = KHI_KRNX_M2MM; }
                else { conv = 1; }

                diff = p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].home*conv - motion_data.ang[jt];
                if ( fabs(diff) > KHI_KRNX_ACTIVATE_TH )
                {
                    is_ready = false;
                    break;
                }
            }

            if ( is_ready )
            {
                /* Speed 100 */
                return_code = krnx_SetMonSpeed( cont_no, ano, 100, &error_code );
                break;
            }
        }
    }

    if ( !conditionCheck( cont_no ) ) { return false; }

    setState( cont_no, ACTIVE );

    return true;
}

bool KhiRobotKrnxDriver::hold( const int cont_no, const JointData joint )
{
    int state;
    bool ret = true;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    state = getState( cont_no );
    if ( state == ACTIVE )
    {
        ret = setState( cont_no, HOLDED );
    }

    return ret;
}

bool KhiRobotKrnxDriver::deactivate( const int cont_no )
{
    char msg[1024] = { 0 };
    int error_lamp = 0;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( in_simulation )
    {
        setState( cont_no, DEACTIVATING );
        setState( cont_no, INACTIVE );
        return true;
    }

    setState( cont_no, DEACTIVATING );

    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        /* Hold Program */
        return_code = krnx_Hold( cont_no, ano, &error_code );
        ros::Duration(0.2).sleep();
        /* Kill Program */
        return_code = krnx_Kill( cont_no, ano, &error_code );
        /* Motor Power OFF */
        return_code = execAsMonCmd( cont_no, "ZPOW OFF", msg_buf, sizeof(msg_buf), &error_code );
        return_code = krnx_GetCurErrorLamp( cont_no, ano, &error_lamp );
        if ( retKrnxRes( cont_no, "krnx_GetCurErrorLamp", return_code ) && ( error_lamp != 0 ) )
        {
            /* Error Reset */
            return_code = krnx_Ereset( cont_no, ano, &error_code );
        }
    }

    setState( cont_no, INACTIVE );

    return true;
}

bool KhiRobotKrnxDriver::loadDriverParam( const int cont_no )
{
    char robot_name[64] = { 0 };
    char msg[256] = { 0 };
    TKrnxPanelInfo panel_info;

    if ( robot_info[cont_no].arm_num <= 0 )
    {
        errorPrint( "Invalid robot size" );
        return false;
    }

    for ( int ano = 0; ano<robot_info[cont_no].arm_num; ano++ )
    {
        /* Robot Name */
        return_code = krnx_GetRobotName( cont_no, ano, robot_name );
        if ( strncmp( robot_name, robot_info[cont_no].robot_name.c_str(), 6 ) != 0 )
        {
            errorPrint( "ROS:%s does not match AS:%s", robot_info[cont_no].robot_name.c_str(), robot_name );
            return false;
        }

        /* AS Switch */
        return_code = execAsMonCmd( cont_no, "TYPE SWITCH(ZDBLREFFLT_MODSTABLE)", msg_buf, sizeof(msg_buf), &error_code );
        sw_dblrefflt[cont_no] = atoi(msg_buf);
        if ( sw_dblrefflt[cont_no] == -1 )
        {
            return_code = execAsMonCmd( cont_no, "SW ZDBLREFFLT_MODSTABLE=OFF", msg_buf, sizeof(msg_buf), &error_code );
        }

        /* KRNX */
        TKrnxRtcInfo rtc_info;
        rtc_info.cyc = (int)(robot_info[cont_no].period/1e+6);
        rtc_info.buf = KHI_KRNX_BUFFER_SIZE;
        rtc_info.interpolation = 1;
        return_code = krnx_SetRtcInfo( cont_no, &rtc_info );
        retKrnxRes( cont_no, "krnx_SetRtcInfo", return_code );
        krnx_SetRtcCompMask( cont_no, ano, pow( 2, p_rb_tbl[cont_no]->arm_tbl[ano].jt_num ) - 1 );

        return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
        if ( retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) && ( panel_info.cycle_lamp != 0 ) )
        {
            /* Hold Program */
            return_code = krnx_Hold( cont_no, ano, &error_code );
            if ( !retKrnxRes( cont_no, "krnx_Hold", return_code ) ) { return false; }
        }

        /* Kill Program */
        return_code = krnx_Kill( cont_no, ano, &error_code );
        if ( !retKrnxRes( cont_no, "krnx_Kill", return_code ) ) { return false; }

        /* Load Program */
        if ( !loadRtcProg( cont_no, p_rb_tbl[cont_no]->robot_name.c_str() ) )
        {
            errorPrint( "Failed to load RTC program");
            return false;
        }
    }

    return true;
}

/**
 * public read function
 */
bool KhiRobotKrnxDriver::readData( const int cont_no, JointData *joint )
{
    static int sim_cnt = 0;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( in_simulation )
    {
        memcpy( &joint->pos[0], &joint->cmd[0], sizeof(joint->cmd) );
        if ( ( sim_cnt - 1 ) % KRNX_PRINT_TH == 0 )
        {
            jointPrint( std::string("read"), *joint );
        }
        sim_cnt++;
        return true;
    }

    static std::vector<TKrnxCurMotionData> motion_data[KRNX_MAX_CONTROLLER][KRNX_MAX_ROBOT];
    TKrnxCurMotionData data[KRNX_MAX_ROBOT];
    float ang[KRNX_MAX_ROBOT][KRNX_MAXAXES] = {{ 0 }};
    float vel[KRNX_MAX_ROBOT][KRNX_MAXAXES] = {{ 0 }};
    int ano, jt;

    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        if ( !getCurMotionData( cont_no, ano, &data[ano] ) ) { return false; }

        if ( motion_data[cont_no][ano].size() >= KRNX_MOTION_BUF )
        {
            motion_data[cont_no][ano].erase( motion_data[cont_no][ano].begin() );
        }
        motion_data[cont_no][ano].push_back( data[ano] );

        // ang
        memcpy( ang[ano], &data[ano].ang, sizeof(data[ano].ang) );
        // vel
        if ( motion_data[cont_no][ano].size() > 1 )
        {
            std::vector<TKrnxCurMotionData>::iterator it = motion_data[cont_no][ano].end();
            it--;
            for ( int jt=0; jt < KHI_MAX_JOINT; jt++ )
            {
                vel[ano][jt] = motion_data[cont_no][ano].back().ang[jt] - it->ang[jt];
            }
        }
    }

    ano = 0;
    jt = 0;
    for ( int cnt = 0; cnt < joint->joint_num; cnt++ )
    {
        joint->pos[cnt] = ang[ano][jt];
        joint->vel[cnt] = vel[ano][jt];
        joint->eff[cnt] = 0; // tmp

        /* [ mm ] to [ m ] */
        if ( p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].type == TYPE_LINE )
        {
            joint->pos[cnt] /= KHI_KRNX_M2MM;
            joint->vel[cnt] /= KHI_KRNX_M2MM;
        }

        jt++;
        if ( jt >= p_rb_tbl[cont_no]->arm_tbl[ano].jt_num )
        {
            jt = 0;
            ano++;
            if ( ano >= p_rb_tbl[cont_no]->arm_num ) { break; }
        }
    }

    return true;
}

bool KhiRobotKrnxDriver::getCurMotionData( const int cont_no, const int robot_no, TKrnxCurMotionData *p_motion_data )
{
    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    return_code = krnx_GetCurMotionData( cont_no, robot_no, p_motion_data );

    return retKrnxRes( cont_no, "krnx_GetCurMotionData", return_code );
}

bool KhiRobotKrnxDriver::setJointDataHome( const int cont_no, JointData *joint )
{
    int ano = 0;
    int jt = 0;

    for ( int cnt = 0; cnt < joint->joint_num; cnt++ )
    {
        joint->cmd[cnt] = p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].home;
        joint->pos[cnt] = p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].home;

        jt++;
        if ( jt >= p_rb_tbl[cont_no]->arm_tbl[ano].jt_num )
        {
            jt = 0;
            ano++;
            if ( ano >= p_rb_tbl[cont_no]->arm_num ) { break; }
        }
    }

    return true;
}

/**
 * public write function
 */
bool KhiRobotKrnxDriver::writeData( const int cont_no, JointData joint )
{
    static int sim_cnt = 0;
    int idx, ano, jt;
    char msg[1024] = { 0 };
    char status[128] = { 0 };
    TKrnxCurMotionData data;
    float jt_pos = 0.0F;
    float jt_vel = 0.0F;
    bool is_primed = true;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( getState( cont_no ) != ACTIVE ) { return true; }

    if ( in_simulation )
    {
        if ( ( sim_cnt - 1 ) % KRNX_PRINT_TH == 0 )
        {
            jointPrint( std::string("write"), joint );
        }
        sim_cnt++;
        return true;
    }

    /* convert */
    /* cnt: JointData's joint no */
    /* jt: ROBOT's joint no */
    ano = 0;
    jt = 0;
    for ( int cnt = 0; cnt < joint.joint_num; cnt++ )
    {
        rtc_comp[cont_no][ano][jt] = joint.cmd[cnt] - p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].home;
        if ( p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].type == TYPE_LINE )
        {
            rtc_comp[cont_no][ano][jt] *= KHI_KRNX_M2MM;
        }

        jt++;
        if ( jt >= p_rb_tbl[cont_no]->arm_tbl[ano].jt_num )
        {
            jt = 0;
            ano++;
            if ( ano >= p_rb_tbl[cont_no]->arm_num ) { break; }
        }
    }

    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        return_code = krnx_PrimeRtcCompData( cont_no, ano, &rtc_comp[cont_no][ano][0], &rtc_status[cont_no][ano][0] );
        if ( !retKrnxRes( cont_no, "krnx_PrimeRtcCompData", return_code ) ) { is_primed = false; }
    }
    if ( !is_primed )
    {
        /* ERROR log */
        for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
        {
            snprintf( msg, sizeof(msg), "[krnx_PrimeRtcCompData] ano:%d [jt]pos:vel:status ", ano+1 );
            krnx_GetRtcCompData( cont_no, ano, &rtc_old_comp[cont_no][ano][0] );
            getCurMotionData( cont_no, ano, &data );
            for ( int jt = 0; jt < p_rb_tbl[cont_no]->arm_tbl[ano].jt_num; jt++ )
            {
                jt_pos = data.ang_ref[jt];
                jt_vel = ( rtc_comp[cont_no][ano][jt] - rtc_old_comp[cont_no][ano][jt] )*(1e+9/robot_info[cont_no].period);
                if ( p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].type == TYPE_LINE )
                {
                    jt_pos /= KHI_KRNX_M2MM;
                    jt_vel /= KHI_KRNX_M2MM;
                }
                snprintf( status, sizeof(status), "[%d]%.4f:%.4f:%d ", jt+1, jt_pos, jt_vel, rtc_status[cont_no][ano][jt] );
                strcat( msg, status );
            }
            errorPrint( msg );
        }
        return false;
    }

    return_code = krnx_SendRtcCompData( cont_no, rtc_seq_no[cont_no] );
    rtc_seq_no[cont_no]++;

    return retKrnxRes( cont_no, "krnx_SendRtcCompData", return_code );
}

bool KhiRobotKrnxDriver::updateState( const int cont_no )
{
    int error_lamp = 0;
    int error_code = 0;
    int rtc_sw = 0;
    int state;
    TKrnxPanelInfo panel_info;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    state = getState( cont_no );
    if ( state == ERROR )
    {
        for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
        {
            return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
            if ( retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) && ( panel_info.cycle_lamp != 0 ) )
            {
                /* Hold Program */
                return_code = krnx_Hold( cont_no, ano, &error_code );
                if ( !retKrnxRes( cont_no, "krnx_Hold", return_code ) ) { return false; }
            }
        }
    }

    if ( in_simulation ) { return true; }

    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        return_code = krnx_GetCurErrorLamp( cont_no, ano, &error_lamp );
        if ( ( state != ERROR ) && ( error_lamp != 0 ) )
        {
            return_code = krnx_GetCurErrorInfo( cont_no, ano, &error_code );
            errorPrint( "AS ERROR %d: ano:%d code:%d", cont_no, ano+1, error_code );
            setState( cont_no, ERROR );
            return true;
        }
        return_code = krnx_GetRtcSwitch( cont_no, ano, &rtc_sw );
        if ( ( state != INACTIVE ) && ( rtc_sw == 0 ) )
        {
            errorPrint( "RTC SWITCH turned OFF %d: ano:%d", cont_no, ano+1 );
            deactivate( cont_no );
            return true;
        }
    }

    return true;
}

bool KhiRobotKrnxDriver::getPeriodDiff( const int cont_no, double *diff )
{
    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }
    static bool buffer_enabled = false;
    int ano = 0;
    int buffer_length = 0;

    if ( getState( cont_no ) != ACTIVE )
    {
        *diff = 0;
        return true;
    }

    if ( in_simulation )
    {
        *diff = 0;
        return true;
    }

    buffer_length = krnx_GetRtcBufferLength( cont_no, ano );
    // check if KRNX can get buffer length from KHI controller
    if ( buffer_length > 0 )
    {
        buffer_enabled = true;
    }

    if ( buffer_enabled )
    {
        *diff = ( buffer_length - KHI_KRNX_BUFFER_SIZE ) * robot_info[cont_no].period;
    }
    else
    {
        *diff = 0;
    }

    return true;
}

std::vector<std::string> KhiRobotKrnxDriver::splitString( const std::string str, const char del )
{
    int first = 0;
    int last = str.find_first_of( del );
    std::vector<std::string> list;

    if ( first < str.size() )
    {
        std::string sub_str1( str, first, last - first );
        list.push_back( sub_str1 );
        first = last + 1;
        std::string sub_str2( str, first, std::string::npos );
        list.push_back( sub_str2 );
    }

    return list;
}

bool KhiRobotKrnxDriver::loadRtcProg( const int cont_no, const std::string name )
{
    FILE *fp;
    int fd;
    char tmplt[] = "/tmp/khi_robot-rtc_param-XXXXXX";
    char fd_path[128] = { 0 };
    char file_path[128] = { 0 };
    ssize_t rsize;

    fd = mkstemp(tmplt);

    if ( ( fp = fdopen( fd, "w" ) ) != NULL)
    {
        /* retrieve path */
        snprintf( fd_path, sizeof(fd_path), "/proc/%d/fd/%d", getpid(), fd );
        rsize = readlink( fd_path, file_path, sizeof(file_path) );
        if ( rsize < 0 ) { return false; }

        /* RTC program */
        if ( name == KHI_ROBOT_WD002N )
        {
            fprintf( fp, ".PROGRAM rb_rtc1()\n" );
            fprintf( fp, "  FOR .i = 1 TO 8\n" );
            fprintf( fp, "    .acc[.i] = 1\n" );
            fprintf( fp, "  END\n" );
            fprintf( fp, "  L3ACCURACY .acc[1] ALWAYS\n" );
            fprintf( fp, "  FOR .i = 1 TO 8\n" );
            fprintf( fp, "    .acc[.i] = 0\n" );
            fprintf( fp, "  END\n" );
            fprintf( fp, "  RTC_SW 1: ON\n" );
            fprintf( fp, "1 JMOVE #rtchome1\n" );
            fprintf( fp, "  GOTO 1\n" );
            fprintf( fp, "  RTC_SW 1: OFF\n" );
            fprintf( fp, ".END\n" );
            fprintf( fp, ".PROGRAM rb_rtc2()\n" );
            fprintf( fp, "  FOR .i = 1 TO 8\n" );
            fprintf( fp, "    .acc[.i] = 1\n" );
            fprintf( fp, "  END\n" );
            fprintf( fp, "  L3ACCURACY .acc[1] ALWAYS\n" );
            fprintf( fp, "  FOR .i = 1 TO 8\n" );
            fprintf( fp, "    .acc[.i] = 0\n" );
            fprintf( fp, "  END\n" );
            fprintf( fp, "  RTC_SW 2: ON\n" );
            fprintf( fp, "1 JMOVE #rtchome2\n" );
            fprintf( fp, "  GOTO 1\n" );
            fprintf( fp, "  RTC_SW 2: OFF\n" );
            fprintf( fp, ".END\n" );
        }
        else
        {
            fprintf( fp, ".PROGRAM rb_rtc1()\n" );
            fprintf( fp, "  ACCURACY 1 FINE\n" );
            fprintf( fp, "  JMOVE #rtchome1\n" );
            fprintf( fp, "  ACCURACY 0 ALWAYS\n" );
            fprintf( fp, "  RTC_SW 1: ON\n" );
            fprintf( fp, "1 JMOVE #rtchome1\n" );
            fprintf( fp, "  GOTO 1\n" );
            fprintf( fp, "  RTC_SW 1: OFF\n" );
            fprintf( fp, ".END\n" );
        }
        fclose( fp );
    }
    else
    {
        return false;
    }

    return_code = krnx_Load( cont_no, file_path );
    unlink( file_path );
    if ( !retKrnxRes( cont_no, "krnx_Load", return_code ) ) { return false; }

    return true;
}

bool KhiRobotKrnxDriver::syncRtcPos( const int cont_no )
{
    TKrnxCurMotionData motion_data = { 0 };

    for ( int ano = 0; ano < robot_info[cont_no].arm_num; ano++ )
    {
        /* AS */
        snprintf( cmd_buf, sizeof(cmd_buf), "HERE/N %d: #rtchome%d", ano+1, ano+1 );
        return_code = execAsMonCmd( cont_no, cmd_buf, msg_buf, sizeof(msg_buf), &error_code );

        /* Driver */
        if ( !getCurMotionData( cont_no, ano, &motion_data ) ) { return false; }
        for ( int jt = 0; jt < p_rb_tbl[cont_no]->arm_tbl[ano].jt_num; jt++ )
        {
            memcpy( &p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].home, &motion_data.ang[jt], sizeof(motion_data.ang[jt]) );
            if ( p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].type == TYPE_LINE )
            {
                p_rb_tbl[cont_no]->arm_tbl[ano].jt_tbl[jt].home /= KHI_KRNX_M2MM;
            }
        }
    }

    return true;
}

bool KhiRobotKrnxDriver::commandHandler( khi_robot_msgs::KhiRobotCmd::Request &req, khi_robot_msgs::KhiRobotCmd::Response &res)
{
    int cont_no = 0;
    char resp[KRNX_MSGSIZE] = { 0 };
    int acode;
    int dcode;
    std::vector<std::string> vlist;
    std::string api_cmd;
    const char del = ' ';
    int arg;
    int onoff;
    int state;
    TKrnxIoInfo io;

    /* default */
    res.driver_ret = KRNX_NOERROR;
    res.as_ret = 0;
    res.cmd_ret = "";
    api_cmd = "";
    arg = 0;
    onoff = 0;

    if ( req.type == "as" )
    {
        if ( !isTransitionState( cont_no ) )
        {
            dcode = execAsMonCmd( cont_no, req.cmd.c_str(), resp, sizeof(resp), &acode );
            res.driver_ret = dcode;
            res.as_ret = acode;
            res.cmd_ret = std::string(resp);
        }
        else
        {
            res.driver_ret = KRNX_E_BADARGS;
            res.cmd_ret = "IS TRANSITION STATE";
        }
    }
    else if ( req.type == "driver" )
    {
        state = getState( cont_no );
        if ( req.cmd == "get_status" )
        {
            res.cmd_ret = getStateName( cont_no );
        }
        else if ( req.cmd == "hold" )
        {
            if ( state == ACTIVE )
            {
                setStateTrigger( cont_no, HOLD );
            }
            else
            {
                res.cmd_ret = "NOT ACTIVE STATE";
            }
        }
        else if ( req.cmd == "restart" )
        {
            if ( ( state == INACTIVE ) || ( state == HOLDED ) || ( state == ERROR ) )
            {
                setStateTrigger( cont_no, RESTART );
            }
            else
            {
                res.cmd_ret = "NOT INACTIVE/HOLDED/ERROR STATE";
            }
        }
        else if ( req.cmd == "quit" )
        {
            setStateTrigger( cont_no, QUIT );
        }
        else
        {
            vlist = splitString( req.cmd, del );
            if ( vlist.size() == 2 )
            {
                api_cmd = vlist[0];
                if ( api_cmd == "get_signal" )
                {
                    dcode = krnx_GetCurIoInfo( cont_no, &io );
                    res.driver_ret = dcode;
                    arg = std::atoi( vlist[1].c_str() );
                    if ( arg >= 1 && arg <= KHI_MAX_SIG_SIZE )
                    {
                        /* DO */
                        onoff = io.io_do[(arg-1)/8] & ( 1 << (arg-1)%8 );
                    }
                    else if ( arg >= 1000 && arg <= 1000 + KHI_MAX_SIG_SIZE )
                    {
                        /* DI */
                        arg -= 1000;
                        onoff = io.io_di[(arg-1)/8] & ( 1 << (arg-1)%8 );
                    }
                    else if ( arg >= 2001 && arg <= 2000 + KHI_MAX_SIG_SIZE )
                    {
                        /* INTERNAL */
                        arg -= 2000;
                        onoff = io.internal[(arg-1)/8] & ( 1 << (arg-1)%8 );
                    }
                    else
                    {
                        res.driver_ret = KRNX_E_BADARGS;
                        res.cmd_ret = "INVALID ARGS";
                    }
                    if ( res.driver_ret == KRNX_NOERROR )
                    {
                        if ( onoff ) { res.cmd_ret = "-1"; }
                        else         { res.cmd_ret = "0";}
                    }
                }
                else if ( api_cmd == "set_signal" )
                {
                    std::string as_cmd = req.cmd;
                    as_cmd.replace( 0, strlen("set_signal"), "SIGNAL" );
                    dcode = execAsMonCmd( cont_no, as_cmd.c_str(), resp, sizeof(resp), &acode );
                    res.driver_ret = dcode;
                    res.as_ret = acode;
                }
                else
                {
                    res.driver_ret = KRNX_E_BADARGS;
                    res.cmd_ret = "INVALID CMD";
                }
            }
            else
            {
                res.driver_ret = KRNX_E_BADARGS;
                res.cmd_ret = "INVALID ARGS";
            }
        }
    }
    else
    {
        res.driver_ret = KRNX_E_BADARGS;
        res.cmd_ret = "INVALID TYPE";
    }

    return true;
}

} // end of khi_robot_control namespace