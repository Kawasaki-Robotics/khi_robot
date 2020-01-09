/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Modified 2016, by Shadow Robot Company Ltd.
 *  Modified 2017, by Tokyo Opensource Robotics Kyokai Association
 *  Modified 2019, by Kawasaki Heavy Industries, LTD.
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

#include <getopt.h>
#include <execinfo.h>
#include <csignal>
#include <pthread.h>
#include <numeric>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <khi_robot_hardware_interface.h>

using boost::accumulators::accumulator_set;
using boost::accumulators::stats;
using boost::accumulators::extract_result;
using boost::accumulators::tag::max;
using boost::accumulators::tag::mean;
using std::string;
using std::vector;
using std::accumulate;
using realtime_tools::RealtimePublisher;

static struct
{
  char *program_;
  bool write_;
  double period_;
  std::string ip_;
  bool simulation_;
  std::string robot_;
}
g_options;

void Usage( const string &msg = "" )
{
    fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
    fprintf(stderr, "  Available options\n");
    fprintf(stderr, "    -i, --ip                    IP address for Controller\n");
    fprintf(stderr, "    -l, --loopback              Use loopback interface for Controller (i.e. simulation mode)\n");
    fprintf(stderr, "    -p, --period                RT loop period in msec\n");
    fprintf(stderr, "    -v, --viewer                Viewing robot through Rviz\n");
    fprintf(stderr, "    -r, --robot                 Robot name\n");
    fprintf(stderr, "    -h, --help                  Print this message and exit\n");
    if ( msg != "" )
    {
        fprintf(stderr, "Error: %s\n", msg.c_str());
        exit(-1);
    }
    else
    {
        exit(0);
    }
}

static int g_quit = 0;
static double last_published, last_loop_start, last_rt_monitor_time;
static const int SEC_2_NSEC = 1e+9;
static const int SEC_2_USEC = 1e+6;
static const double PERIOD_SLEEP = 0.1; /* 100msec */
static const double PERIOD_DIFF_WEIGHT = 1e-2;

static struct
{
    accumulator_set<double, stats<max, mean> > read_acc;
    accumulator_set<double, stats<max, mean> > write_acc;
    accumulator_set<double, stats<max, mean> > loop_acc;
    accumulator_set<double, stats<max, mean> > jitter_acc;
    int overruns;
    int recent_overruns;        /* overruns during publishing */
    int last_overrun;           /* diagnostics count after last overruns occurred */
    int last_severe_overrun;    /* diagnostics severe count after last overruns occurred */
    double overrun_loop_sec;
    double overrun_read;
    double overrun_write;

    /* These values are set when realtime loop does not meet performance expectations */
    bool rt_loop_not_making_timing;
    double halt_rt_loop_frequency;
    double rt_loop_frequency;
}
g_stats;

static void publishDiagnostics(RealtimePublisher<diagnostic_msgs::DiagnosticArray>& publisher)
{
    if ( publisher.trylock() )
    {
        accumulator_set<double, stats<max, mean> > zero;
        vector<diagnostic_msgs::DiagnosticStatus> statuses;
        diagnostic_updater::DiagnosticStatusWrapper status;

        static double max_read = 0, max_write = 0, max_loop = 0, max_jitter = 0;
        double avg_read, avg_write, avg_loop, avg_jitter;

        avg_read = extract_result<mean>(g_stats.read_acc);
        avg_write = extract_result<mean>(g_stats.write_acc);
        avg_loop = extract_result<mean>(g_stats.loop_acc);
        max_read = std::max(max_read, extract_result<max>(g_stats.read_acc));
        max_write = std::max(max_write, extract_result<max>(g_stats.write_acc));
        max_loop = std::max(max_loop, extract_result<max>(g_stats.loop_acc));
        g_stats.read_acc = zero;
        g_stats.write_acc = zero;
        g_stats.loop_acc = zero;

        /* Publish average loop jitter */
        avg_jitter = extract_result<mean>(g_stats.jitter_acc);
        max_jitter = std::max(max_jitter, extract_result<max>(g_stats.jitter_acc));
        g_stats.jitter_acc = zero;

        status.addf("Max READ roundtrip (us)", "%.2f", max_read * SEC_2_USEC);
        status.addf("Avg READ roundtrip (us)", "%.2f", avg_read * SEC_2_USEC);
        status.addf("Max WRITE roundtrip (us)", "%.2f", max_write * SEC_2_USEC);
        status.addf("Avg WRITE roundtrip (us)", "%.2f", avg_write * SEC_2_USEC);
        status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop * SEC_2_USEC);
        status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop * SEC_2_USEC);
        status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * SEC_2_USEC);
        status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * SEC_2_USEC);
        status.addf("Control Loop Overruns", "%d", g_stats.overruns);
        status.addf("Recent Control Loop Overruns", "%d", g_stats.recent_overruns);
        status.addf("Last Control Loop Overrun Cause", "READ: %.2fus, WRITE: %.2fus",
                    g_stats.overrun_read*SEC_2_USEC, g_stats.overrun_write * SEC_2_USEC);
        status.addf("Last Overrun Loop Time (us)", "%.2f", g_stats.overrun_loop_sec * SEC_2_USEC);
        status.addf("Realtime Loop Frequency", "%.4f", g_stats.rt_loop_frequency);

        status.name = "Realtime Control Loop";
        if (g_stats.overruns > 0 && g_stats.last_overrun < 30)
        {
          if (g_stats.last_severe_overrun < 30)
            status.level = 1;
          else
            status.level = 0;
          status.message = "Realtime loop used too much time in the last 30 seconds.";
        }
        else
        {
          status.level = 0;
          status.message = "OK";
        }
        g_stats.recent_overruns = 0;
        ++g_stats.last_overrun;
        ++g_stats.last_severe_overrun;

        if (g_stats.rt_loop_not_making_timing)
          status.mergeSummaryf(status.ERROR, "realtime loop only ran at %.4f Hz", g_stats.halt_rt_loop_frequency);

        statuses.push_back(status);
        publisher.msg_.status = statuses;
        publisher.msg_.header.stamp = ros::Time::now();
        publisher.unlockAndPublish();
    }
}

static inline double now()
{
    struct timespec n;
    clock_gettime( CLOCK_MONOTONIC, &n );
    return static_cast<double>(n.tv_nsec) / SEC_2_NSEC + n.tv_sec;
}

static void timespecInc( struct timespec& tick, const int& nsec )
{
    tick.tv_nsec += nsec;
    if ( tick.tv_nsec > 0 )
    {
        while (tick.tv_nsec >= SEC_2_NSEC)
        {
            tick.tv_nsec -= SEC_2_NSEC;
            ++tick.tv_sec;
        }
    }
    else
    {
        while (tick.tv_nsec < 0)
        {
            tick.tv_nsec += SEC_2_NSEC;
            --tick.tv_sec;
        }
    }
}

class RTLoopHistory
{
public:
    RTLoopHistory(unsigned length, double default_value) :
        index_(0),
        length_(length),
        history_(length, default_value)
    {
    }

    void sample(double value)
    {
        index_ = (index_ + 1) % length_;
        history_[index_] = value;
    }

    double average() const
    {
        return accumulate(history_.begin(), history_.end(), 0.0) / static_cast<double>(length_);
    }

protected:
    unsigned index_;
    unsigned length_;
    vector<double> history_;
};

inline bool activate( khi_robot_control::KhiRobotHardwareInterface& robot, struct timespec* tick )
{
    bool ret = true;

    if ( g_options.write_ )
    {
        ret = robot.activate();
        if ( ret == false )
        {
            ROS_ERROR( "Failed to activate KHI robot" );
        }
    }

    clock_gettime( CLOCK_REALTIME, tick );

    /* Snap to the nearest second */
    tick->tv_nsec = (tick->tv_nsec / g_options.period_ + 1) * g_options.period_;
    clock_nanosleep( CLOCK_REALTIME, TIMER_ABSTIME, tick, NULL );

    last_published = now();
    last_rt_monitor_time = now();
    last_loop_start = now();

    return ret;
}

void quitRequested( int sig )
{
    ROS_WARN( "received signal %d", sig );
    g_quit = 1;
}

void *controlLoop( void* )
{
    ros::NodeHandle nh;
    int state_trigger = khi_robot_control::NONE;

    /* Catch attempts to quit */
    signal( SIGTERM, quitRequested );
    signal( SIGINT, quitRequested );
    signal( SIGHUP, quitRequested );

    /* Diagnostics */
    RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher( nh, "/diagnostics", 2 );

    /* Realtime Loop Frequency */
    const double rt_loop_monitor_period = 0.2; /* Calculate realtime loop frequency every 200msec */
    const double period_in_secs = 1e+9 * g_options.period_;
    const double given_frequency = 1 / period_in_secs;
    double min_acceptable_rt_loop_frequency = 0.75 * given_frequency;
    if ( nh.getParam("min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency ) )
    {
        ROS_WARN("min_acceptable_rt_loop_frequency changed to %f", min_acceptable_rt_loop_frequency);
    }
    unsigned rt_cycle_count = 0;
    RTLoopHistory rt_loop_history( 3, 1000.0 ); /* Keep history of last 3 calculation intervals. */

    /* Realtime Scheduler */
    struct sched_param thread_param;
    int policy = SCHED_FIFO;
    thread_param.sched_priority = sched_get_priority_max( policy );
    if ( pthread_setschedparam( pthread_self(), policy, &thread_param ) == 0 )
    {
        ROS_INFO( "KHI robot control started. [REALTIME]" );
    }
    else
    {
        ROS_INFO( "KHI robot control started. [NOT REALTIME]" );
    }

    /* Publish one-time before entering real-time to pre-allocate message vectors */
    publishDiagnostics( publisher );

    /* Initialization */
    struct timespec tick;
    ros::Duration durp( g_options.period_ / 1e+9 );
    khi_robot_control::KhiRobotHardwareInterface robot;
    controller_manager::ControllerManager cm(&robot);
    double period_diff = 0;
    if ( !robot.open( g_options.robot_, g_options.ip_, g_options.period_, g_options.simulation_ ) )
    {
        ROS_ERROR( "Failed to open KHI robot" );
        publisher.stop();
        robot.close();
        delete &robot;
        ros::shutdown();
        return NULL;
    }
    if ( !activate( robot, &tick ) )
    {
        publisher.stop();
        robot.deactivate();
        robot.close();
        delete &robot;
        ros::shutdown();
        return NULL;
    }

    while ( !g_quit )
    {
        double this_loop_start = now();
        g_stats.loop_acc( this_loop_start - last_loop_start );
        last_loop_start = this_loop_start;

        double start = now();

        ros::Time this_moment( tick.tv_sec, tick.tv_nsec );
        robot.read( this_moment, durp );

        double after_read = now();

        cm.update( this_moment, durp );
        if ( g_options.write_ )
        {
            /* Robot State */
            robot.updateState();
            state_trigger = robot.getStateTrigger();
            if ( state_trigger == khi_robot_control::HOLD )
            {
                robot.hold();
                continue;
            }
            else if ( state_trigger == khi_robot_control::RESTART )
            {
                if ( activate( robot, &tick ) )
                {
                    ros::Time activate_moment( tick.tv_sec, tick.tv_nsec );
                    robot.read( activate_moment, durp );
                    cm.update( activate_moment, durp, true );
                }
                continue;
            }
            else if ( state_trigger == khi_robot_control::QUIT )
            {
                g_quit = true;
                continue;
            }

            robot.write( this_moment, durp );
        }

        /* Cycle Adjustment */
        if ( robot.getPeriodDiff( period_diff ) )
        {
            timespecInc( tick, PERIOD_DIFF_WEIGHT * period_diff );
        }

        double end = now();

        g_stats.read_acc( after_read - start );
        g_stats.write_acc( end - after_read );

        if ( ( end - last_published ) > 1.0 )
        {
            publishDiagnostics( publisher );
            last_published = end;
         }

        /* Check Realtime Loop Frequency */
        ++rt_cycle_count;
        if ( ( start - last_rt_monitor_time ) > rt_loop_monitor_period )
        {
            /* Calculate new average rt loop frequency */
            double rt_loop_frequency = static_cast<double>(rt_cycle_count) / rt_loop_monitor_period;

            /* Use last X samples of frequency when deciding whether or not to halt */
            rt_loop_history.sample(rt_loop_frequency);
            double avg_rt_loop_frequency = rt_loop_history.average();
            if ( avg_rt_loop_frequency < min_acceptable_rt_loop_frequency )
            {
                if ( !g_stats.rt_loop_not_making_timing )
                {
                    /* Only update this value if motors when this first occurs (used for diagnostics error message) */
                    g_stats.halt_rt_loop_frequency = avg_rt_loop_frequency;
                }
                g_stats.rt_loop_not_making_timing = true;
            }
            g_stats.rt_loop_frequency = avg_rt_loop_frequency;
            rt_cycle_count = 0;
            last_rt_monitor_time = start;
        }

        /* Compute end of next g_options.period_ */
        timespecInc( tick, g_options.period_ );

        /* Check Overrun */
        struct timespec before;
        clock_gettime( CLOCK_REALTIME, &before );
        if ( ( before.tv_sec + static_cast<double>(before.tv_nsec) / SEC_2_NSEC) >
             ( tick.tv_sec + static_cast<double>(tick.tv_nsec) / SEC_2_NSEC ) )
        {
            /* Total amount of time the loop took to run */
            g_stats.overrun_loop_sec = ( before.tv_sec + static_cast<double>(before.tv_nsec) / SEC_2_NSEC ) -
                                       ( tick.tv_sec + static_cast<double>(tick.tv_nsec) / SEC_2_NSEC );

            /* We overran, snap to next "g_options.period_" */
            tick.tv_sec = before.tv_sec;
            tick.tv_nsec = ( before.tv_nsec / g_options.period_ ) * g_options.period_;
            timespecInc( tick, g_options.period_ );

            /* Initialize overruns */
            if ( g_stats.overruns == 0 )
            {
                g_stats.last_overrun = 1000;
                g_stats.last_severe_overrun = 1000;
            }
            /* Check for overruns */
            if ( g_stats.recent_overruns > 10 )
            {
                g_stats.last_severe_overrun = 0;
            }
            g_stats.last_overrun = 0;

            ++g_stats.overruns;
            ++g_stats.recent_overruns;
            g_stats.overrun_read = after_read - start;
            g_stats.overrun_write = end - after_read;
        }

        /* Sleep until end of g_options.period_ */
        clock_nanosleep( CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL );

        /* Check Jitter */
        struct timespec after;
        clock_gettime( CLOCK_REALTIME, &after );
        double jitter = ( after.tv_sec - tick.tv_sec + static_cast<double>(after.tv_nsec - tick.tv_nsec) / SEC_2_NSEC );
        g_stats.jitter_acc( jitter );
    }
    publisher.stop();
    robot.deactivate();
    robot.close();
    delete &robot;
    ROS_INFO( "KHI robot control ended." );
    ros::shutdown();
    return NULL;
}

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;

int main(int argc, char *argv[])
{
    /* Initialize ROS and parse command-line arguments */
    ros::init( argc, argv, "KhiRobotControl" );

    /* Options */
    g_options.program_ = argv[0];
    g_options.period_ = 4e+6;  /* 4ms */
    g_options.simulation_ = false;
    g_options.write_ = true;

    while (true)
    {
        static struct option long_options[] =
        {
            {"help", no_argument, 0, 'h'},
            {"interface", required_argument, 0, 'i'},
            {"loopback", no_argument, 0, 'l'},
            {"viewer", no_argument, 0, 'v'},
            {"period", required_argument, 0, 'p'},
            {"robot", required_argument, 0, 'r'},
        };
        int option_index = 0;
        int c = getopt_long( argc, argv, "hi:lvp:r:", long_options, &option_index );
        if (c == -1)
        {
            break;
        }

        switch (c)
        {
          case 'h':
            Usage();
            break;
          case 'i':
            g_options.ip_ = std::string(optarg);
            break;
          case 'l':
            g_options.simulation_ = true;
            break;
          case 'v':
            ROS_INFO( "Viewer mode" );
            g_options.write_ = false;
            break;
          case 'p':
            /* convert period given in msec to nsec */
            g_options.period_ = fabs(atof(optarg))*1e+6;
            break;
          case 'r':
            g_options.robot_ = std::string(optarg);
            break;
          default:
            break;
        }
    }

    if ( optind < argc )
    {
        Usage( "Extra arguments" );
    }

    /* Start controlLoop thread */
    int rv = pthread_create( &controlThread, &controlThreadAttr, controlLoop, 0 );
    if ( rv != 0 )
    {
        ROS_FATAL( "Unable to create control thread: rv = %d", rv );
        exit( EXIT_FAILURE );
    }

    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    pthread_join(controlThread, reinterpret_cast<void **>(&rv));
    ros::waitForShutdown();
    return rv;
}

