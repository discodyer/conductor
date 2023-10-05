/**
 * @file robocup.cpp
 * @brief RoboCup2023，使用Conductor封装库
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
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

#include "conductor/mission_state.h"
#include "conductor/ansi_color.h"
#include "conductor/apm.h"
#include "conductor/fixed_point.h"
#include "conductor/waypoint.h"
#include "conductor/yolo.h"
#include "conductor/dropper.h"

#include <string.h>
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
    ros::init(argc, argv, "robocup_guided_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, safeSigintHandler);

    double startup_delay = 10.0;
    bool is_test = false;
    PidParams pidpara_armor{0.17, 0.0, 0.015, // kp, ki, kd
                            10.0, 40, 0.0};   // windup_guard, output_bound, sample_time

    std::string transform_json_path = "./src/conductor/example/json/transforms.json"; // 修改为你的 JSON 文件路径

    std::string waypoint_json_path = "./src/conductor/example/json/waypoints.json"; // 修改为你的 JSON 文件路径

    // Read parameters from launch file, including: transformJsonFilePath, waypointJsonFilePath
    {
        if (nh.getParam("/robocup_guided_node/transform_json_path", transform_json_path))
        {
            ROS_INFO("Get transform json path parameter: %s", transform_json_path.c_str());
        }
        else
        {
            ROS_WARN("Using default transform json path: %s", transform_json_path.c_str());
        }

        if (nh.getParam("/robocup_guided_node/waypoint_json_path", waypoint_json_path))
        {
            ROS_INFO("Get waypoint json path parameter: %s", waypoint_json_path.c_str());
        }
        else
        {
            ROS_WARN("Using default waypoint json path: %s", waypoint_json_path.c_str());
        }

        if (nh.getParam("/robocup_guided_node/startup_delay", startup_delay))
        {
            ROS_INFO("Get startup_delay parameter: %f", startup_delay);
        }
        else
        {
            ROS_WARN("Using default startup_delay: %f", startup_delay);
        }

        if (nh.getParam("/robocup_guided_node/is_test", is_test))
        {
            ROS_INFO("Get is_test parameter: %s", is_test ? "true" : "false");
        }
        else
        {
            ROS_WARN("Using default is_test: %s", is_test ? "true" : "false");
        }

        if (nh.getParam("/robocup_guided_node/armor_kp", pidpara_armor.kp))
        {
            ROS_INFO("Get armor_kp parameter: %f", pidpara_armor.kp);
        }
        else
        {
            ROS_WARN("Using default armor_kp: %f", pidpara_armor.kp);
        }

        if (nh.getParam("/robocup_guided_node/armor_kd", pidpara_armor.kd))
        {
            ROS_INFO("Get armor_kd parameter: %f", pidpara_armor.kd);
        }
        else
        {
            ROS_WARN("Using default armor_kd: %f", pidpara_armor.kd);
        }
    }

    // 创建 FrameManager 实例，并从 JSON 文件读取坐标变换数据
    FrameManager transformManager(transform_json_path);

    // 创建 WaypointManager 实例，并从 JSON 文件读取航点数据
    WaypointManager waypointManager(waypoint_json_path);

    // 输出坐标变换
    while (ros::ok())
    {
        transformManager.printFrameInfoALL();
        break;
    }

    Dropper dropper(nh);

    ArduConductor apm(nh, 20.0);

    // wait for FCU connection
    while (ros::ok() && !apm.current_state.connected && !is_test)
    {
        if (is_interrupted)
        {
            ros::shutdown();
            return 0;
        }
        ros::spinOnce();
        apm.rate.sleep();
    }
    if (apm.current_state.connected)
    {
        ROS_INFO(SUCCESS("Drone connected!"));
    }

    // 检测 from body to camera_init 的变换是否存在
    // 如果不存在就说明lio没有输出
    ROS_INFO("Checking transform from body to camera_init...");
    while (ros::ok() && !is_test)
    {
        if (is_interrupted)
        {
            ros::shutdown();
            return 0;
        }
        if (!transformManager.isWorldFrameExist())
        {
            ROS_INFO("Wait for Lio");
            ros::spinOnce();
            apm.rate.sleep();
        }
        else
        {
            // 发布transform文件中的静态坐标变换
            ROS_INFO("Publishing Frame...");
            transformManager.publishFrameAll();
            break;
        }
    }

    // 等待接收起飞指令
    ROS_INFO("Waiting for Takeoff Command");
    while (ros::ok() && !dropper.is_allow_takeoff)
    {
        if (is_interrupted)
        {
            ros::shutdown();
            return 0;
        }
        ros::spinOnce();
        apm.rate.sleep();
    }

    // 测试退出点
    if (is_test)
    {
        ROS_INFO("END TEST, Exiting....");
        ros::shutdown();
        return false;
    }

    // startup delay 起飞延时
    ros::Duration(startup_delay).sleep();

    FixedPointYolo fixedPointArmor("filter_targets",   // 订阅话题
                                   "tent",             // 订阅Yolo标签
                                   {640 / 2, 640 / 2}, // 目标点
                                   50.0,               // 锁定判定距离
                                   nh,                 // 节点句柄
                                   pidpara_armor,      // PID参数列表 x
                                   pidpara_armor,      // PID参数列表 y
                                   100                 // 锁定计数
    );

    waypoint::Waypoint waypoint_snapshot_break;             // 中断时航点
    waypoint::Waypoint waypoint_snapshot_break_destination; // 中断时目标航点

    waypoint::Waypoint waypoint_snapshot_target;                    // 目标点悬停位置（参考摄像头中心）
    waypoint::Waypoint waypoint_snapshot_drop_waypoint;             // 目标投放位置
    waypoint::Position waypoint_dropper_1_offset{0.2, -0.3, -0.08}; // 1号左侧投掷物偏移
    waypoint::Position waypoint_dropper_2_offset{0.1, 0.0, 0.3};    // 2号中间投掷物偏移
    waypoint::Position waypoint_dropper_3_offset{0.0, 0.1, 0.3};    // 3号右侧投掷物偏移

    // 重置上一次操作的时间为当前时刻
    apm.last_request = ros::Time::now();
    int count = 0;
    while (ros::ok() && !is_interrupted)
    {
        // 任务执行状态机
        switch (apm.mission_state)
        {
        case MissionState::kPrearm:
            if (apm.setModeGuided(5.0)) // 修改飞行模式为 Guided (ArduCopter)
            {
                apm.sendGpOrigin();     // 如果切换成Guided模式就发送全局原点坐标
                apm.setMoveSpeed(0.15); // 设置空速
            }
            break;

        case MissionState::kArm:
            apm.arm(5.0); // 解锁电机
            break;

        case MissionState::kTakeoff:
            if (apm.takeoff(0.5, 1.0, 3.0)) // 起飞到1m高度
            {
                apm.mission_state = MissionState::kPose;
                ros::Duration(2.0).sleep();
                ROS_INFO(MISSION_SWITCH_TO("pose"));
                waypointManager.resetDelayTime();
            }

            break;

        case MissionState::kPose:
            if (!waypointManager.getCurrentTargetWaypoint().is_published) // 发布下一个航点
            {
                waypointManager.printCurrentWaypoint();
                apm.setPoseWorld(
                    transformManager.getWorldWaypoint(
                        waypointManager.getCurrentTargetWaypoint()));
                waypointManager.setCurrentTargetWaypointIsPublished(true);
            }

            if (waypointManager.isWaypointDelaySatisfied() || // 判断航点是否超时
                waypoint::calculateDistance(
                    transformManager.getCurrentPoseWorld(),
                    transformManager.getWorldWaypoint(
                        waypointManager.getCurrentTargetWaypoint())) <
                    waypointManager.getCurrentTargetWaypoint().range) // 判断是否到达目标点附近
            {
                if (!waypointManager.goToNextWaypoint())
                {
                    ROS_INFO(SUCCESS("Finish Loading Waypoint !"));
                    apm.updateLastRequestTime();
                    apm.mission_state = MissionState::kLand;
                }
            }

            if (fixedPointArmor.is_found_ && !fixedPointArmor.is_dropped_) // 如果检测到目标点
            {
                apm.setBreak();          // 悬停
                fixedPointArmor.clear(); // 重置PID控制器
                waypoint_snapshot_break_destination = transformManager.getWorldWaypoint(
                    waypointManager.getCurrentTargetWaypoint());                  // 记录当前目标航点
                waypoint_snapshot_break = transformManager.getCurrentPoseWorld(); // 记录中断位置
                apm.mission_state = MissionState::kTarget;                        // 状态机切换
            }
            break;

        case MissionState::kTarget:
            if (!fixedPointArmor.is_lock_counter_reached() && !fixedPointArmor.is_locked_)
            {
                apm.setSpeedBody(fixedPointArmor.getBoundedOutput().x * 0.01,
                                 fixedPointArmor.getBoundedOutput().y * 0.01,
                                 0, 0);
                break;
            }

            if (fixedPointArmor.is_lock_counter_reached() && !fixedPointArmor.is_locked_)
            {
                apm.setBreak();
                fixedPointArmor.is_locked_ = true;
                ROS_INFO("is_lock_counter_reached");
                // 记录目标点悬停位置（参考摄像头中心）
                waypoint_snapshot_target = transformManager.getCurrentPoseWorld();
                // 移动到偏移位置
                waypoint_snapshot_drop_waypoint = waypoint_snapshot_target;
                waypoint_snapshot_drop_waypoint.position.x += waypoint_dropper_1_offset.x;
                waypoint_snapshot_drop_waypoint.position.y += waypoint_dropper_1_offset.y;
                waypoint_snapshot_drop_waypoint.position.z = waypoint_dropper_1_offset.z;
                waypoint_snapshot_drop_waypoint.air_speed = 0.1;
                apm.setPoseWorld(waypoint_snapshot_drop_waypoint);
                apm.updateLastRequestTime();
                break;
            }

            if (fixedPointArmor.is_locked_ &&
                !fixedPointArmor.is_dropped_ &&
                (apm.isTimeElapsed(5.0) ||
                 (waypoint::calculateDistance(
                      transformManager.getCurrentPoseWorld(),
                      waypoint_snapshot_drop_waypoint) <=
                  0.05))) // 判断是否到达目标点附近
            {
                ROS_INFO("is_dropped_");
                apm.setBreak();    // 悬停
                dropper.dropAll(); // 投掷左侧
                fixedPointArmor.is_dropped_ = true;
                apm.mission_state = MissionState::kWayback; // 状态机切换
                apm.updateLastRequestTime();
            }
            else
            {
                ROS_INFO(COLORED_TEXT("distance: %0.2f", ANSI_BACKGROUND_RED), waypoint::calculateDistance(
                                                                                   transformManager.getCurrentPoseWorld(),
                                                                                   waypoint_snapshot_drop_waypoint));
            }

            break;

        case MissionState::kWayback:
            apm.setMoveSpeed(0.2); // 设置空速
            apm.setPoseWorld(0.0, 0.0, 0.5, 0.0);
            apm.mission_state = MissionState::kLand; // 状态机切换
            apm.updateLastRequestTime();
            break;

        case MissionState::kLand:
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
