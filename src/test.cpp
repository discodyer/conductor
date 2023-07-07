/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <conductor/mission_state.hpp>
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
    // ros::shutdown();
    // 设置中断标志
    is_interrupted = true;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guided_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, safeSigintHandler);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    mission_state drone_state = prearm;

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    // send a few setpoints before starting
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode set_mode_guided;
    set_mode_guided.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 1;

    ros::Time last_request = ros::Time::now();

    while (ros::ok() && !is_interrupted)
    {
        switch (drone_state)
        {
        case prearm:
            if (current_state.mode != "GUIDED" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(set_mode_guided) &&
                    set_mode_guided.response.mode_sent)
                {
                    ROS_INFO("Guided enabled");
                    drone_state = arm;
                }
                last_request = ros::Time::now();
            }
            else if (current_state.mode == "GUIDED" &&
                     (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                ROS_INFO("Guided enabled");
                drone_state = arm;
                last_request = ros::Time::now();
            }
            break;

        case arm:
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed!");
                    drone_state = takeoff;
                }
                else
                {
                    ROS_INFO("Vehicle arm failed!");
                    drone_state = prearm;
                }
                last_request = ros::Time::now();
            }
            break;

        case takeoff:

            if (takeoff_client.call(takeoff_cmd) &&
                (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                ROS_INFO("Vehicle Takeoff!");
                drone_state = land;
            }
            else
            {
                ROS_INFO("Vehicle Takeoff Failed!");
            }
            if (!current_state.armed)
            {
                drone_state = arm;
            }
            last_request = ros::Time::now();

            break;

        case land:
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                mavros_msgs::CommandTOL land_cmd;
                if (land_client.call(land_cmd) &&
                    land_cmd.response.success)
                {
                    ROS_INFO("Vehicle landed!");
                    drone_state = prearm;
                    ros::shutdown();
                }
                last_request = ros::Time::now();
            }
            break;

        default:
            break;
        }

        ros::spinOnce();
        rate.sleep();
        // 在循环结束后调用ros::shutdown()
        ros::shutdown();
    }

    return 0;
}
