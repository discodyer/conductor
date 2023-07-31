/**
 * @file set_gp_origin.cc
 * @brief 设置地理坐标原点
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
#include <geometry_msgs/Point.h>

#include "conductor/mission_state.h"
#include "conductor/ansi_color.h"
#include "conductor/apm.h"

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
        ROS_INFO(COLORED_TEXT("Vehicle landed!", ANSI_COLOR_GREEN));
    }
    // 设置中断标志
    is_interrupted = true;
    // 不能在这边执行ros::shutdown();要在循环结束后执行，这边应该设置中断标记让主循环结束
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guided_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, safeSigintHandler);

    ArduConductor apm(nh);

    // wait for FCU connection
    while (ros::ok() && !apm.current_state.connected && !is_interrupted)
    {
        if (is_interrupted)
        {
            ros::shutdown();
            return 0;
        }
        ros::spinOnce();
        apm.rate.sleep();
    }

    ROS_INFO(SUCCESS("Drone connected!"));

    // 重置上一次操作的时间为当前时刻
    apm.last_request = ros::Time::now();
    int count = 0;
    while (ros::ok() && !is_interrupted)
    {
        if (apm.setModeGuided(5.0)) // 修改飞行模式为 Guided (ArduCopter)
        {
            apm.sendGpOrigin();     // 如果切换成Guided模式就发送全局原点坐标
            apm.setMoveSpeed(0.15); // 设置空速
            break;
        }

        ros::spinOnce();
        apm.rate.sleep();
    }
    // 在循环结束后调用ros::shutdown()
    ros::shutdown();
    return 0;
}
