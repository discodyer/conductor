#ifndef WAY_POINT_H
#define WAY_POINT_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>

namespace way_point
{
    // 航点类型枚举型
    enum class WaypointType
    {
        kPoseWorld, // 全局坐标航点
        kPoseBody,  // 相对坐标航点
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
        std::string from_frame; // 起始坐标系
        std::string to_frame;   // 目标坐标系
        Position translation;   // 平移向量
        Quaternion orientation; // 四元数表示的旋转
    };

    // 航点结构体
    struct Waypoint
    {
        size_t index;      // 航点索引
        WaypointType type; // 航点类型
        Position position; // 航点坐标
        double yaw;        // 航点朝向
        double delay;      // 航点延时，单位：秒
        double air_speed;  // 空速
    };

}

class WaypointManager
{
public:
    WaypointManager(const std::string &jsonFilePath);

    // 获取并转到下一个航点
    bool getNextWaypoint(way_point::Waypoint &waypoint);

    void resetDelayTime();
    
    // 转到下一个航点
    bool goToNextWaypoint();

    // 获取当前航点信息
    way_point::Waypoint getCurrentWaypoint() const;

    // 判断是否满足航点延时
    bool isWaypointDelaySatisfied() const;

    // 获取当前航点类型
    way_point::WaypointType getCurrentWaypointType() const;

    // 输出当前航点的信息
    void printCurrentWaypoint() const;
    void printCurrentWaypointLoop();

    bool is_current_waypoint_published_;

private:
    std::vector<way_point::Waypoint> waypoints_;
    size_t current_waypoint_index_;
    ros::Time last_waypoint_time_;
};

#endif // WAY_POINT_H
