#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Range.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <queue>

// 本地坐标点结构体
struct local_point
{
    double x;   // x坐标
    double y;   // y坐标
    double z;   // z坐标
    double yaw; // 偏航角
};

// PD控制器类
class PD_Controller
{
public:
    double Kp_x; // 比例增益 - X轴
    double Kd_x; // 微分增益 - X轴
    double Kp_y; // 比例增益 - Y轴
    double Kd_y; // 微分增益 - Y轴

    PD_Controller(double kp_x, double kd_x, double kp_y, double kd_y)
    {
        Kp_x = kp_x;
        Kd_x = kd_x;
        Kp_y = kp_y;
        Kd_y = kd_y;
    }

    // 计算输出
    geometry_msgs::Point compute(const geometry_msgs::Point &error)
    {
        geometry_msgs::Point output;
        output.x = Kp_x * error.x - Kd_x * error.x;
        output.y = Kp_y * error.y - Kd_y * error.y;
        output.z = 0.0; // 只进行平面控制
        return output;
    }
};

// 全局变量
mavros_msgs::State current_state;                 // 当前状态
geometry_msgs::PoseStamped current_pose;          // 当前姿态
geometry_msgs::Point current_point;               // 当前点（摄像头向下）
geometry_msgs::Point current_point2;              // 当前点（摄像头朝前）
std_msgs::UInt8 barcode_value;                    // 条形码值
std_msgs::Bool laser_help;                        // 激光辅助
bool pos_ok = false;                              // 位置是否准确
bool color_green_gray = false;                    // 颜色是否为绿色或灰色
std::queue<local_point> waypoints;                // 航点队列
tf2_ros::Buffer tfBuffer;                         // TF缓冲区
ros::Publisher set_raw_pub;                       // 发布器 - 设置原始位置目标
ros::Publisher local_pos_pub;                     // 发布器 - 设置本地位置目标
ros::Publisher set_gp_origin_pub;                 // 发布器 - 设置地理坐标原点
ros::Publisher laser_help_pub;                    // 发布器 - 激光辅助
ros::Publisher mode_pub;                          // 发布器 - 摄像头模式（向下）
ros::Publisher mode_pub2;                         // 发布器 - 摄像头模式（朝前）
ros::Publisher led_pub;                           // 发布器 - LED灯控制
ros::Publisher mode_led_pub;                      // 发布器 - 模式LED灯控制
ros::Subscriber currentPos;                       // 订阅器 - 当前位置
ros::Subscriber state_sub;                        // 订阅器 - 状态
ros::Subscriber point_sub;                        // 订阅器 - 摄像头向下点云数据
ros::Subscriber point_sub2;                       // 订阅器 - 摄像头朝前点云数据
ros::Subscriber color_sub;                        // 订阅器 - 颜色
ros::Subscriber bar_sub;                          // 订阅器 - 条形码
ros::Subscriber front_dis_sub;                    // 订阅器 - 前方距离传感器
ros::Subscriber key_sub;                           // 订阅器 - 按键

// 设置航点相对于Point A的位置
void set_pose_PointA(float x, float y, float z, float yaw);

// 发送全局位置原点
void send_gp_origin(ros::NodeHandle &nh);

// 解锁无人机
int arm_drone(ros::NodeHandle &nh);

// 设置为GUIDED模式
int set_guided(ros::NodeHandle &);
// 设置航点相对于Point A的位置
void set_pose_PointA(float x, float y, float z, float yaw)
{
    // 创建一个姿态消息
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "local_origin";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, yaw)));

    // 发布航点位置消息
    local_pos_pub.publish(pose);
}

// 发送全局位置原点
void send_gp_origin(ros::NodeHandle &nh)
{
    geographic_msgs::GeoPointStamped origin;
    origin.header.stamp = ros::Time::now();
    origin.header.frame_id = "local_origin";
    origin.position.latitude = 0;
    origin.position.longitude = 0;
    origin.position.altitude = 0;

    set_gp_origin_pub.publish(origin);
}

// 解锁无人机
int arm_drone(ros::NodeHandle &nh)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (!nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming").call(arm_cmd) || !arm_cmd.response.success)
    {
        ROS_ERROR("Failed to arm the drone");
        return -1;
    }
    return 0;
}

// 设置为GUIDED模式
int set_guided(ros::NodeHandle &nh)
{
    mavros_msgs::SetMode guided_set_mode;
    guided_set_mode.request.custom_mode = "GUIDED";
    if (!nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode").call(guided_set_mode) || !guided_set_mode.response.mode_sent)
    {
        ROS_ERROR("Failed to set the mode to GUIDED");
        return -1;
    }
    return 0;
}

// 起飞
int takeoff(ros::NodeHandle &nh, float height)
{
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = height;
    if (!nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff").call(takeoff_cmd) || !takeoff_cmd.response.success)
    {
        ROS_ERROR("Failed to takeoff");
        return -1;
    }
    return 0;
}

// 降落
int land(ros::NodeHandle &nh)
{
    mavros_msgs::CommandTOL land_cmd;
    if (!nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land").call(land_cmd) || !land_cmd.response.success)
    {
        ROS_ERROR("Failed to land");
        return -1;
    }
    return 0;
}

// 设置速度（机体坐标系）
void set_speed_body(float vx, float vy, float vz, float yaw_rate)
{
    mavros_msgs::PositionTarget set_speed;
    set_speed.header.stamp = ros::Time::now();
    set_speed.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    set_speed.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW |
                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    set_speed.velocity.x = vx;
    set_speed.velocity.y = vy;
    set_speed.velocity.z = vz;
    set_speed.yaw_rate = yaw_rate;

    set_raw_pub.publish(set_speed);
}

// 设置速度（局部坐标系）
void set_speed_local(float vx, float vy, float vz, float yaw_rate)
{
    mavros_msgs::PositionTarget set_speed;
    set_speed.header.stamp = ros::Time::now();
    set_speed.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    set_speed.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW |
                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    set_speed.velocity.x = vx;
    set_speed.velocity.y = vy;
    set_speed.velocity.z = vz;
    set_speed.yaw_rate = yaw_rate;

    set_raw_pub.publish(set_speed);
}

// 设置本地位置
void set_pose_local(float x, float y, float z, float yaw)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "local_origin";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, yaw)));

    local_pos_pub.publish(pose);
}

// 设置机体坐标系下的姿态
void set_pose_body(float x, float y, float z, float yaw)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "local_origin";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, yaw)));

    local_pos_pub.publish(pose);
}

// 设置摄像头向下的模式
void set_camera_down_mode(int mode)
{
    std_msgs::Int8 mode_msg;
    mode_msg.data = mode;
    mode_pub.publish(mode_msg);
}

// 设置摄像头朝前的模式
void set_camera_front_mode(int mode)
{
    std_msgs::Int8 mode_msg;
    mode_msg.data = mode;
    mode_pub2.publish(mode_msg);
}

// 设置LED灯状态
void set_led_state(uint8_t state)
{
    std_msgs::UInt8 led_msg;
    led_msg.data = state;
    led_pub.publish(led_msg);
}

// 边界限制函数
template <typename T>
T bound(T val, T limit)
{
    if (val > limit)
        return limit;
    else if (val < -limit)
        return -limit;
    else
        return val;
}

#endif  // DRONE_CONTROL_H
