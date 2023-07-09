/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float64.h>

#include "conductor/mission_state.hpp"
// #include "conductor/apm.hpp"
#include "conductor/base.hpp"

#include "signal.h" //necessary for the Custom SIGINT handler
#include "stdio.h"  //necessary for the Custom SIGINT handler

bool is_interrupted = false;

// 定义一个安全的SIGINT处理函数
void safeSigintHandler(int sig)
{
    // 创建一个临时的节点句柄
    ros::NodeHandle nh;
    // 创建一个降落服务客户端
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    // 停止运动并降落
    mavros_msgs::CommandTOL land_cmd;
    if (land_client.call(land_cmd) &&
        land_cmd.response.success)
    {
        ROS_INFO("Vehicle landed!");
    }
    // 设置中断标志
    is_interrupted = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guided_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    BaseConductor apm(&nh);

    signal(SIGINT, safeSigintHandler);

    // wait for FCU connection
    while (ros::ok() && !apm.current_state.connected)
    {
        ros::spinOnce();
        apm.rate.sleep();
    }

    apm.last_request = ros::Time::now();

    while (ros::ok() && !is_interrupted)
    {
        switch (apm.mission_state)
        {
        case prearm:
            apm.set_mode_guided(5.0);
            break;

        case arm:
            apm.arm(5.0);
            break;

        case takeoff:
            if (apm.takeoff(1.0))
            {
                apm.mission_state = land;
                ROS_INFO("Mission mode -> land");
            }

            break;

        case land:
            if (apm.land(10.0))
            {
                ros::shutdown();
            }
            break;

        default:
            break;
        }

        ros::spinOnce();
        apm.rate.sleep();
    }
    // 在循环结束后调用ros::shutdown()
    ros::shutdown();
    return 0;
}
