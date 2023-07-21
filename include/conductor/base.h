#ifndef BASE_H
#define BASE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float64.h>

#include "conductor/mission_state.h"

class BaseConductor
{
protected:
    virtual void initNode();
    ros::NodeHandle nh_;                    // ROS 节点句柄

    ros::Subscriber state_sub_;             // 订阅器 - 状态

    ros::Publisher local_pos_pub_;          // 发布器 - 设置本地位置目标
    ros::Publisher set_raw_pub_;            // 发布器 - 设置原始位置目标

    void subStateCallback(const mavros_msgs::State::ConstPtr &msg);

public:
    BaseConductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0);
    BaseConductor(ros::NodeHandle *nodehandle, double rate = 20);

    MissionState mission_state;

    ros::Time last_request;
    ros::Rate rate;
    mavros_msgs::State current_state;

    bool setModeGuided(double delay = 5.0);
    bool arm(double delay = 5.0);
    bool takeoff(double altitude, double delay = 3.0);
    bool land(double delay = 10.0);

    bool isTimeElapsed(double delay);
    void updateLastRequestTime();

    ~BaseConductor(){};
};

#endif // BASE_H