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
    PidParams pidpara{0.17, 0.0, 0.015, // kp, ki, kd
                      10.0, 40, 0.0};   // windup_guard, output_bound, sample_time

    std::string transform_json_path = "./src/conductor/example/json/transforms.json";

    std::string waypoint_json_path_1 = "./src/conductor/example/json/waypoints.json";
    std::string waypoint_json_path_2 = "./src/conductor/example/json/waypoints.json";
    std::string waypoint_json_path_3 = "./src/conductor/example/json/waypoints.json";
    std::string waypoint_json_path_4 = "./src/conductor/example/json/waypoints.json";

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

        if (nh.getParam("/robocup_guided_node/waypoint_json_path_1", waypoint_json_path_1))
        {
            ROS_INFO("Get waypoint json path 1 parameter: %s", waypoint_json_path_1.c_str());
        }
        else
        {
            ROS_WARN("Using default waypoint 1 json path: %s", waypoint_json_path_1.c_str());
        }

        if (nh.getParam("/robocup_guided_node/waypoint_json_path_2", waypoint_json_path_2))
        {
            ROS_INFO("Get waypoint json path 2 parameter: %s", waypoint_json_path_2.c_str());
        }
        else
        {
            ROS_WARN("Using default waypoint 2 json path: %s", waypoint_json_path_2.c_str());
        }

        if (nh.getParam("/robocup_guided_node/waypoint_json_path_3", waypoint_json_path_3))
        {
            ROS_INFO("Get waypoint json path 3 parameter: %s", waypoint_json_path_3.c_str());
        }
        else
        {
            ROS_WARN("Using default waypoint 3 json path: %s", waypoint_json_path_3.c_str());
        }

        if (nh.getParam("/robocup_guided_node/waypoint_json_path_4", waypoint_json_path_4))
        {
            ROS_INFO("Get waypoint json path 4 parameter: %s", waypoint_json_path_4.c_str());
        }
        else
        {
            ROS_WARN("Using default waypoint 4 json path: %s", waypoint_json_path_4.c_str());
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
    }

    // 创建 FrameManager 实例，并从 JSON 文件读取坐标变换数据
    FrameManager transformManager(transform_json_path);

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

    dropper.takeoffAck();
    dropper.beep();
    std::string waypoint_json_path;

    switch (dropper.takeoff_mode_)
    {
    case DropperTakeoffMode::kNormal:
        waypoint_json_path = waypoint_json_path_1;
        break;
    case DropperTakeoffMode::kFallBack:
        waypoint_json_path = waypoint_json_path_2;
        break;
    case DropperTakeoffMode::kTakeoff1:
        waypoint_json_path = waypoint_json_path_3;
        break;
    case DropperTakeoffMode::kTakeoff2:
        waypoint_json_path = waypoint_json_path_4;
        break;
    default:
        break;
    }

    ROS_INFO(SUCCESS("using waypoint json : %s"), waypoint_json_path.c_str());
    // 创建 WaypointManager 实例，并从 JSON 文件读取航点数据
    WaypointManager waypointManager(waypoint_json_path);

    // 测试退出点
    if (is_test)
    {
        ROS_INFO("END TEST, Exiting....");
        ros::shutdown();
        return false;
    }

    // startup delay 起飞延时
    ros::Duration(startup_delay).sleep();

    // 装甲车
    FixedPointYolo fixedPointArmor("filter_targets",   // 订阅话题
                                   "armor",            // 订阅Yolo标签
                                   {640 / 2, 640 / 2}, // 目标点
                                   50.0,               // 锁定判定距离
                                   nh,                 // 节点句柄
                                   pidpara,            // PID参数列表 x
                                   pidpara,            // PID参数列表 y
                                   50                  // 锁定计数
    );

    // 桥梁
    FixedPointYolo fixedPointBridge("filter_targets",   // 订阅话题
                                    "bridge",           // 订阅Yolo标签
                                    {640 / 2, 640 / 2}, // 目标点
                                    50.0,               // 锁定判定距离
                                    nh,                 // 节点句柄
                                    pidpara,            // PID参数列表 x
                                    pidpara,            // PID参数列表 y
                                    50                  // 锁定计数
    );

    // 地堡
    FixedPointYolo fixedPointFort("filter_targets",   // 订阅话题
                                  "fort",             // 订阅Yolo标签
                                  {640 / 2, 640 / 2}, // 目标点
                                  50.0,               // 锁定判定距离
                                  nh,                 // 节点句柄
                                  pidpara,            // PID参数列表 x
                                  pidpara,            // PID参数列表 y
                                  50                  // 锁定计数
    );

    waypoint::Waypoint waypoint_snapshot_break;             // 中断时航点
    waypoint::Waypoint waypoint_snapshot_break_destination; // 中断时目标航点

    waypoint::Waypoint waypoint_snapshot_target;        // 目标点悬停位置（参考摄像头中心）
    waypoint::Waypoint waypoint_snapshot_drop_waypoint; // 目标投放位置

    waypoint::Position waypoint_dropper_1_offset{0.2, -0.3, -0.08}; // 1号左侧投掷物偏移
    waypoint::Position waypoint_dropper_2_offset{0.35, 0.0, -0.08}; // 2号中间投掷物偏移
    waypoint::Position waypoint_dropper_3_offset{0.2, 0.3, -0.08};  // 3号右侧投掷物偏移

    // 重置上一次操作的时间为当前时刻
    apm.last_request = ros::Time::now();
    int return_pose_count = 0;
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
                ros::Duration(1.0).sleep();
                ROS_INFO(MISSION_SWITCH_TO("pose"));
                waypointManager.resetDelayTime();
            }

            break;

        case MissionState::kPose:
            if (!waypointManager.getCurrentTargetWaypoint().is_published) // 发布下一个航点
            {
                ROS_INFO(SUCCESS("发布下一个航点"));
                waypointManager.printCurrentWaypoint();
                apm.setMoveSpeed(waypointManager.getCurrentTargetWaypoint().air_speed);
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
                ROS_INFO(SUCCESS("当前航点到达或超时"));
                ROS_INFO(SUCCESS("发布下一个航点"));

                if (!waypointManager.goToNextWaypoint())
                {
                    ROS_INFO(SUCCESS("Finish Loading Waypoint !"));
                    apm.updateLastRequestTime();
                    apm.mission_state = MissionState::kLand;
                    break;
                }
            }

            if (fixedPointArmor.is_found_ && !fixedPointArmor.is_dropped_) // 如果检测到目标点Armor
            {
                ROS_INFO(SUCCESS("检测到目标点Armor"));

                apm.setBreak();          // 悬停
                fixedPointArmor.clear(); // 重置PID控制器
                waypoint_snapshot_break_destination = transformManager.getWorldWaypoint(
                    waypointManager.getCurrentTargetWaypoint());                  // 记录当前目标航点
                waypoint_snapshot_break = transformManager.getCurrentPoseWorld(); // 记录中断位置
                apm.mission_state = MissionState::kTargetArmor;                   // 状态机切换
                break;
            }

            if (fixedPointBridge.is_found_ && !fixedPointBridge.is_dropped_) // 如果检测到目标点Bridge
            {
                ROS_INFO(SUCCESS("检测到目标点Bridge"));

                apm.setBreak();           // 悬停
                fixedPointBridge.clear(); // 重置PID控制器
                waypoint_snapshot_break_destination = transformManager.getWorldWaypoint(
                    waypointManager.getCurrentTargetWaypoint());                  // 记录当前目标航点
                waypoint_snapshot_break = transformManager.getCurrentPoseWorld(); // 记录中断位置
                apm.mission_state = MissionState::kTargetBridge;                  // 状态机切换
                break;
            }

            if (fixedPointFort.is_found_ && !fixedPointFort.is_dropped_) // 如果检测到目标点Fort
            {
                ROS_INFO(SUCCESS("检测到目标点Fort"));

                apm.setBreak();         // 悬停
                fixedPointFort.clear(); // 重置PID控制器
                waypoint_snapshot_break_destination = transformManager.getWorldWaypoint(
                    waypointManager.getCurrentTargetWaypoint());                  // 记录当前目标航点
                waypoint_snapshot_break = transformManager.getCurrentPoseWorld(); // 记录中断位置
                apm.mission_state = MissionState::kTargetFort;                    // 状态机切换
                break;
            }

            // if (fixedPointArmor.is_dropped_ && fixedPointBridge.is_dropped_ && fixedPointFort.is_dropped_)
            // {
            //     ROS_INFO(SUCCESS("三个都投掷成功，回到初始位置"));
            //     apm.updateLastRequestTime();
            //     apm.mission_state = MissionState::kWaybackHome;
            // }
            break;

        case MissionState::kTargetArmor:
            if (fixedPointArmor.isTimeout() && !fixedPointArmor.is_locked_)
            {
                ROS_INFO(SUCCESS("Armor锁定超时"));
                fixedPointArmor.reset();
                fixedPointArmor.timeout_count_++;
                if (fixedPointArmor.timeout_count_ >= 3)
                {
                    fixedPointArmor.is_dropped_ = true;
                    dropper.drop(1); // 投掷左侧
                }
                return_pose_count = 0;
                apm.mission_state = MissionState::kReturnPose; // 状态机切换
                apm.updateLastRequestTime();
                break;
            }
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
                ROS_INFO(SUCCESS("Armor已锁定"));
                // 记录目标点悬停位置（参考摄像头中心）
                waypoint_snapshot_target = transformManager.getCurrentPoseWorld();
                // 移动到偏移位置
                waypoint_snapshot_drop_waypoint = waypoint_snapshot_target;
                waypoint_snapshot_drop_waypoint.position.x += waypoint_dropper_1_offset.x;
                waypoint_snapshot_drop_waypoint.position.y += waypoint_dropper_1_offset.y;
                waypoint_snapshot_drop_waypoint.position.z = waypoint_dropper_1_offset.z;
                waypoint_snapshot_drop_waypoint.air_speed = 0.1;
                apm.setMoveSpeed(0.1);
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
                ROS_INFO(SUCCESS("Armor已投掷左侧"));

                apm.setBreak();  // 悬停
                dropper.drop(1); // 投掷左侧
                fixedPointArmor.is_dropped_ = true;
                return_pose_count = 0;
                apm.mission_state = MissionState::kReturnPose; // 状态机切换
                apm.updateLastRequestTime();
            }
            else
            {
                ROS_INFO(
                    COLORED_TEXT(
                        "distance: %0.2f",
                        ANSI_BACKGROUND_RED),
                    waypoint::calculateDistance(
                        transformManager.getCurrentPoseWorld(),
                        waypoint_snapshot_drop_waypoint));
            }

            break;

        case MissionState::kTargetBridge:
            if (fixedPointBridge.isTimeout() && !fixedPointBridge.is_locked_)
            {
                ROS_INFO(SUCCESS("Bridge锁定超时"));
                fixedPointBridge.reset();
                fixedPointBridge.timeout_count_++;
                if (fixedPointBridge.timeout_count_ >= 3)
                {
                    fixedPointBridge.is_dropped_ = true;
                    dropper.drop(2); // 投掷中间
                }
                return_pose_count = 0;
                apm.mission_state = MissionState::kReturnPose; // 状态机切换
                apm.updateLastRequestTime();
                break;
            }
            if (!fixedPointBridge.is_lock_counter_reached() && !fixedPointBridge.is_locked_)
            {
                apm.setSpeedBody(fixedPointBridge.getBoundedOutput().x * 0.01,
                                 fixedPointBridge.getBoundedOutput().y * 0.01,
                                 0, 0);
                break;
            }

            if (fixedPointBridge.is_lock_counter_reached() && !fixedPointBridge.is_locked_)
            {
                apm.setBreak();
                fixedPointBridge.is_locked_ = true;
                ROS_INFO(SUCCESS("Bridge已锁定"));
                // 记录目标点悬停位置（参考摄像头中心）
                waypoint_snapshot_target = transformManager.getCurrentPoseWorld();
                // 移动到偏移位置
                waypoint_snapshot_drop_waypoint = waypoint_snapshot_target;
                waypoint_snapshot_drop_waypoint.position.x += waypoint_dropper_2_offset.x;
                waypoint_snapshot_drop_waypoint.position.y += waypoint_dropper_2_offset.y;
                waypoint_snapshot_drop_waypoint.position.z = waypoint_dropper_2_offset.z;
                waypoint_snapshot_drop_waypoint.air_speed = 0.1;
                apm.setMoveSpeed(0.1);
                apm.setPoseWorld(waypoint_snapshot_drop_waypoint);
                apm.updateLastRequestTime();
                break;
            }

            if (fixedPointBridge.is_locked_ &&
                !fixedPointBridge.is_dropped_ &&
                (apm.isTimeElapsed(5.0) ||
                 (waypoint::calculateDistance(
                      transformManager.getCurrentPoseWorld(),
                      waypoint_snapshot_drop_waypoint) <=
                  0.05))) // 判断是否到达目标点附近
            {
                ROS_INFO(SUCCESS("Bridge已投掷中间"));
                apm.setBreak();  // 悬停
                dropper.drop(2); // 投掷中间
                fixedPointBridge.is_dropped_ = true;
                return_pose_count = 0;
                apm.mission_state = MissionState::kReturnPose; // 状态机切换
                apm.updateLastRequestTime();
            }
            else
            {
                ROS_INFO(
                    COLORED_TEXT(
                        "distance: %0.2f",
                        ANSI_BACKGROUND_RED),
                    waypoint::calculateDistance(
                        transformManager.getCurrentPoseWorld(),
                        waypoint_snapshot_drop_waypoint));
            }

            break;

        case MissionState::kTargetFort:
            if (fixedPointFort.isTimeout() && !fixedPointFort.is_locked_)
            {
                ROS_INFO(SUCCESS("Fort锁定超时"));
                fixedPointFort.reset();
                fixedPointFort.timeout_count_++;
                if (fixedPointFort.timeout_count_ >= 3)
                {
                    fixedPointFort.is_dropped_ = true;
                    dropper.drop(3); // 投掷左侧
                }
                return_pose_count = 0;
                apm.mission_state = MissionState::kReturnPose; // 状态机切换
                apm.updateLastRequestTime();
                break;
            }

            if (!fixedPointFort.is_lock_counter_reached() && !fixedPointFort.is_locked_)
            {
                apm.setSpeedBody(fixedPointFort.getBoundedOutput().x * 0.01,
                                 fixedPointFort.getBoundedOutput().y * 0.01,
                                 0, 0);
                break;
            }

            if (fixedPointFort.is_lock_counter_reached() && !fixedPointFort.is_locked_)
            {
                apm.setBreak();
                fixedPointFort.is_locked_ = true;
                ROS_INFO(SUCCESS("Fort已锁定"));
                // 记录目标点悬停位置（参考摄像头中心）
                waypoint_snapshot_target = transformManager.getCurrentPoseWorld();
                // 移动到偏移位置
                waypoint_snapshot_drop_waypoint = waypoint_snapshot_target;
                waypoint_snapshot_drop_waypoint.position.x += waypoint_dropper_3_offset.x;
                waypoint_snapshot_drop_waypoint.position.y += waypoint_dropper_3_offset.y;
                waypoint_snapshot_drop_waypoint.position.z = waypoint_dropper_3_offset.z;
                waypoint_snapshot_drop_waypoint.air_speed = 0.1;
                apm.setMoveSpeed(0.1);
                apm.setPoseWorld(waypoint_snapshot_drop_waypoint);
                apm.updateLastRequestTime();
                break;
            }

            if (fixedPointFort.is_locked_ &&
                !fixedPointFort.is_dropped_ &&
                (apm.isTimeElapsed(5.0) ||
                 (waypoint::calculateDistance(
                      transformManager.getCurrentPoseWorld(),
                      waypoint_snapshot_drop_waypoint) <=
                  0.05))) // 判断是否到达目标点附近
            {
                ROS_INFO(SUCCESS("Fort已投掷"));
                apm.setBreak();  // 悬停
                dropper.drop(3); // 投掷左侧
                fixedPointFort.is_dropped_ = true;
                return_pose_count = 0;
                apm.mission_state = MissionState::kReturnPose; // 状态机切换
                apm.updateLastRequestTime();
            }
            else
            {
                ROS_INFO(
                    COLORED_TEXT(
                        "distance: %0.2f",
                        ANSI_BACKGROUND_RED),
                    waypoint::calculateDistance(
                        transformManager.getCurrentPoseWorld(),
                        waypoint_snapshot_drop_waypoint));
            }

            break;

        case MissionState::kReturnPose:
            // 先悬停
            if (return_pose_count == 0)
            {
                ROS_INFO(SUCCESS("回到锁定前位置0"));

                apm.setBreak();
                apm.updateLastRequestTime();
                return_pose_count = 1;
                break;
            }
            // 悬停一秒钟后，回到原来的位置
            if (return_pose_count == 1 && apm.isTimeElapsed(1))
            {
                ROS_INFO(SUCCESS("回到锁定前位置1"));
                apm.setMoveSpeed(0.5);
                apm.setPoseWorld(waypoint_snapshot_break);
                apm.updateLastRequestTime();
                return_pose_count = 2;
                break;
            }
            // 等待4秒钟后，飞到原来的目标点并回到pose模式
            if (return_pose_count == 2 && apm.isTimeElapsed(3))
            {
                ROS_INFO(SUCCESS("回到锁定前位置2"));
                apm.setMoveSpeed(0.5);
                apm.setPoseWorld(
                    transformManager.getWorldWaypoint(
                        waypointManager.getCurrentTargetWaypoint()));
                apm.updateLastRequestTime();
                waypointManager.resetDelayTime();
                return_pose_count = 3;
                apm.mission_state = MissionState::kPose;
                break;
            }

            break;

        case MissionState::kWaybackHome:
            // apm.setMoveSpeed(0.5); // 设置空速
            // apm.setPoseWorld(0.0, 0.0, 1.0, 0.0);
            apm.mission_state = MissionState::kLand; // 状态机切换
            apm.updateLastRequestTime();
            break;

        case MissionState::kLand:
            if (apm.land(10.0)) // 10s后降落
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
