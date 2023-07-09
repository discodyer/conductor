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
private:
    void init_node();
    ros::NodeHandle _nh;

    ros::Subscriber _state_sub;

    ros::Publisher _local_pos_pub;
    ros::Publisher _pos_pub;
    ros::Publisher _vel_pub;
    ros::Publisher _set_raw_pub;

    ros::ServiceClient _arming_client;
    ros::ServiceClient _set_mode_client;
    ros::ServiceClient _takeoff_client;
    ros::ServiceClient _land_client;

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
    bool takeoff(double altitude);
    bool land(double delay = 10.0);

    ~BaseConductor(){};
};

#endif // BASE_HPP