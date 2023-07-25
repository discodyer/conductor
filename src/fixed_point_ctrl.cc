/**
 * @file fixed_point_ctrl.cpp
 * @brief 定点测试，使用Conductor封装库
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
#include "conductor/fixed_point.h"

#include "signal.h" //necessary for the Custom SIGINT handler
#include "stdio.h"  //necessary for the Custom SIGINT handler

#define PI 3.1415926535

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

    PidParams pidpara{0.17, 0.0, 0.015, 10.0, 40, 0.0};

    // nh.getParam("kp", pidpara.kp);
    // nh.getParam("ki", pidpara.ki);
    // nh.getParam("kd", pidpara.kd);
    // nh.getParam("windup_guard", pidpara.windup_guard);
    // nh.getParam("output_bound", pidpara.output_bound);
    // nh.getParam("sample_time", pidpara.sample_time);

    ArduConductor apm(nh);
    FixedPoint fixed_point_red("filter_out",
                               {800 / 2, 330}, nh,
                               pidpara,
                               pidpara);

    // wait for FCU connection
    while (ros::ok() && !apm.current_state.connected && !is_interrupted)
    {
        ros::spinOnce();
        apm.rate.sleep();
    }

    ROS_INFO(SUCCESS("Drone connected!"));

    // 重置上一次操作的时间为当前时刻
    apm.last_request = ros::Time::now();
    int count = 0;
    while (ros::ok() && !is_interrupted)
    {
        // 任务执行状态机
        switch (apm.mission_state)
        {
        case kPrearm:
            if (apm.setModeGuided(5.0)) // 修改飞行模式为 Guided (ArduCopter)
            {
                apm.sendGpOrigin();    // 如果切换成Guided模式就发送全局原点坐标
                apm.setMoveSpeed(0.15); // 设置空速
            }
            break;

        case kArm:
            apm.arm(5.0); // 解锁电机
            break;

        case kTakeoff:
            if (apm.takeoff(0.5)) // 起飞到1M高度
            {
                apm.mission_state = kPose;
                ROS_INFO(MISSION_SWITCH_TO("pose"));
            }

            break;

        case kPose:
            if (apm.isTimeElapsed(2.0) && count == 0)
            {
                apm.setPoseBody(0, 0, 0.6, 0);
                ROS_INFO("takeoff to 1.0");
                fixed_point_red.clear();
                count++;
            }
            else if (apm.isTimeElapsed(4.0) && count == 1)
            {
                // apm.setSpeedBody(1, 0, 0, 0);
                // apm.setMoveSpeed(0.3); // 设置空速
                // apm.setPoseBody(2.70, 0.5, 0, 0);
                // ROS_INFO("to 2.7, 0.5");

                apm.setSpeedBody(fixed_point_red.getBoundedOutput().x * 0.01, fixed_point_red.getBoundedOutput().y * 0.01, 0, 0);

                if (apm.isTimeElapsed(20))
                {
                    apm.setSpeedBody(0, 0, 0, 0);
                    count++;
                }
            }
            else if (apm.isTimeElapsed(20.0))
            {
                apm.last_request = ros::Time::now();
                apm.setSpeedBody(0, 0, 0, 0);
                apm.mission_state = kLand;
            }
            break;

        case kLand:
            if (apm.land(5.0)) // 10s后降落
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
