#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>

#include <eigen3/Eigen/Eigen>
#include <std_msgs/UInt8.h>
#include <quadrotor_msgs/Px4State.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{

    int core_id = 5;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) 
    {
        std::cerr << "Failed to set CPU affinity for thread: px4ctrl "<< std::endl;
    } 
    else 
    {
        std::cout << "Successfully set CPU affinity to core " << core_id << std::endl;
    }

    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    Controller controller(param);
    PX4CtrlFSM fsm(param, controller);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                            10,
                                            boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                    10,
                                                    boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                            100,
                                            boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                        100,
                                                        boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                        ros::VoidConstPtr(),
                                                        ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                        100,
                                        boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                        ros::VoidConstPtr(),
                                        ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    // if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    // {
    //     rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
    //                                                 10,
    //                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    // }
 
    rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                10,
                                                boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                    100,
                                                    boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());
    //kdkd at home
    ros::Subscriber at_home_sub =
                    nh.subscribe<std_msgs::Bool>("/planning/at_home",
                                                    100,
                                                    boost::bind(&At_Home_Data_t::feed, &fsm.at_home_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());
    ros::Subscriber arm_cmd_sub =
                    nh.subscribe<quadrotor_msgs::ArmCmd>("/arm_cmd",
                                                    100,
                                                    boost::bind(&Arm_data_t::feed, &fsm.arm_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());
    ros::Subscriber cancle_mission_sub = 
                    nh.subscribe<geometry_msgs::PointStamped>("/reset_point",
                                                    100,
                                                    boost::bind(&Cancle_Misssion_data_t::feed, &fsm.cancle_mission_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());

    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    // kdkd ui
    fsm.ui_back_home_pub_ = nh.advertise<std_msgs::Int8>("/State", 10);

    // kdkd ui
    fsm.go_home_pub_ = nh.advertise<std_msgs::Empty>("/go_home_trigger", 10);
    fsm.takeoff_height_pub_ = nh.advertise<std_msgs::Float32>("/relative_takeoff_height",10);
    //kdkd finish one inspection
    fsm.px4_state_pub = nh.advertise<quadrotor_msgs::Px4State>("/px4_state",10);

    // fsm.tragger_pub_ = nh.advertise<quadrotor_msgs::Instruction>("/bridge/Instruct", 10);

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
