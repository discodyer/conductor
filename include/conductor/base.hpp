#ifndef BASE_HPP
#define BASE_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float64.h>

#include "conductor/mission_state.hpp"

class BaseConductor
{
protected:
    virtual void init_node();
    ros::NodeHandle _nh;                    // ROS 节点句柄

    ros::Subscriber _state_sub;             // 订阅器 - 状态

    ros::Publisher _local_pos_pub;          // 发布器 - 设置本地位置目标
    ros::Publisher _set_raw_pub;            // 发布器 - 设置原始位置目标

    void sub_state_cb(const mavros_msgs::State::ConstPtr &msg);

public:
    BaseConductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0);
    BaseConductor(ros::NodeHandle *nodehandle, double rate = 20);

    MissionState mission_state;

    ros::Time last_request;
    ros::Rate rate;
    mavros_msgs::State current_state;

    bool set_mode_guided(double delay = 5.0);
    bool arm(double delay = 5.0);
    bool takeoff(double altitude, double delay = 3.0);
    bool land(double delay = 10.0);

    bool is_time_elapsed(double delay);
    void update_last_request_time();

    ~BaseConductor(){};
};

#endif // BASE_HPP