#ifndef DROPPER_H
#define DROPPER_H
#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

enum class DropperTakeoffMode
{
    kForbid,
    kNormal,
    kFallBack,
    kTakeoff1,
    kTakeoff2,
};

class Dropper
{
public:
    Dropper(ros::NodeHandle &nh);
    ~Dropper();
    void takeoffAck();
    void beep();
    void dropAll();
    void pickAll();
    void drop(int num);
    bool is_allow_takeoff;
    DropperTakeoffMode takeoff_mode_;

private:
    void init();
    ros::NodeHandle nh_; // ROS 节点句柄
    ros::Publisher drop_pub_;
    ros::Subscriber takeoff_sub_; // 订阅器 - 起飞指令
    ros::Publisher takeoff_pub_; // 发布器 - 起飞确认
    ros::Publisher beep_pub_; // 发布器 - 蜂鸣器
    void subTakeoffCallback(const std_msgs::String::ConstPtr &msg);
};

#endif // DROPPER_H