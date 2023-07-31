#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace waypoint
{
    // 航点类型枚举型
    enum class WaypointType
    {
        kPoseAbsolute, // 全局坐标航点
        kPoseRelated,  // 相对坐标航点
        kSpecial,   // 特殊航点，需要执行子状态机任务
    };

    // 三维向量模板类
    template <typename T>
    struct Vector3_
    {
        T x;
        T y;
        T z;
    };
    typedef Vector3_<double> Position;

    // 坐标变换关系结构体
    struct FrameTransform
    {
        std::string source_frame_id; // 子坐标系
        std::string target_frame_id; // 父坐标系
        tf2::Vector3 translation;    // 位置变换
        tf2::Quaternion rotation;    // 方向变换
    };

    // 航点结构体
    struct Waypoint
    {
        size_t index;         // 航点索引
        WaypointType type;    // 航点类型
        std::string frame_id; // 航点所属坐标系
        Position position;    // 航点坐标
        double yaw;           // 航点朝向
        double delay;         // 航点延时，单位：秒
        double air_speed;     // 空速
    };
}

class WaypointManager
{
public:
    WaypointManager(const std::string &jsonFilePath);

    // 获取并转到下一个航点
    bool getNextWaypoint(waypoint::Waypoint &waypoint);

    void resetDelayTime();

    // 转到下一个航点
    bool goToNextWaypoint();

    // 获取当前航点信息
    waypoint::Waypoint getCurrentWaypoint() const;

    // 判断是否满足航点延时
    bool isWaypointDelaySatisfied() const;

    // 获取当前航点类型
    waypoint::WaypointType getCurrentWaypointType() const;

    // 输出当前航点的信息
    void printCurrentWaypoint() const;
    void printCurrentWaypointLoop();

    bool is_current_waypoint_published_;

private:
    std::vector<waypoint::Waypoint> waypoints_;
    size_t current_waypoint_index_;
    ros::Time last_waypoint_time_;
};

class FrameManager
{
public:
    FrameManager(const std::string &jsonFilePath);

    // 判断世界坐标系是否准备完毕
    bool isWorldFrameExist() const;
    
    // 判断是否存在坐标系
    bool isFrameExist(const std::string &target_frame_id) const;

    bool isTargetExist(const waypoint::FrameTransform &frame) const;

    // 发布坐标关系
    void publishFrame(const waypoint::FrameTransform &frame);

    void publishFrameAll();

private:
    // 静态坐标转换广播器
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<waypoint::FrameTransform> frameTransforms_; // 存储坐标系转换关系
};

#endif // WAYPOINT_H
