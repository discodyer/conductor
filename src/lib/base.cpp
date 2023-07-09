#include "conductor/base.hpp"

BaseConductor::BaseConductor(int &argc, char **argv, const std::string &name, double rate_num, uint32_t options): rate(rate_num)
{
    ros::init(argc, argv, name, options);
    mission_state = prearm;
    this->init_node();
}

BaseConductor::BaseConductor(ros::NodeHandle *nodehandle, double rate_num) : rate(rate_num), _nh(*nodehandle)
{
    mission_state = prearm;
    this->init_node();
}

void BaseConductor::init_node()
{
    _state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, &BaseConductor::sub_state_cb, this);
    _local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    _takeoff_client = _nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    _pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("pos_cmd", 1);
    _vel_pub = _nh.advertise<std_msgs::Float64>("vel_cmd", 1);
    _set_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
}

void BaseConductor::sub_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    this->current_state = *msg;
}


bool BaseConductor::set_mode_guided(double delay)
{
    mavros_msgs::SetMode mode_guided_msg;
    mode_guided_msg.request.custom_mode = "GUIDED";
    if (ros::Time::now() - this->last_request > ros::Duration(delay))
    {
        if (_set_mode_client.call(mode_guided_msg) &&
            mode_guided_msg.response.mode_sent)
        {
            ROS_INFO("Guided enabled!");
            this->mission_state = MissionState::arm;
            ROS_INFO("Mission mode -> arm");
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_INFO("Guided Switch Failed!");
            this->mission_state = MissionState::prearm;
            ROS_INFO("Mission mode -> prearm");
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

bool BaseConductor::arm(double delay)
{
    if (!current_state.armed &&
        (ros::Time::now() - this->last_request > ros::Duration(delay)))
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (_arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
            ROS_INFO("Vehicle armed!");
            this->mission_state = MissionState::takeoff;
            ROS_INFO("Mission mode -> takeoff");
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_INFO("Vehicle arm failed!");
            this->mission_state = MissionState::prearm;
            ROS_INFO("Mission mode -> prearm");
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

bool BaseConductor::takeoff(double altitude)
{
    if (ros::Time::now() - this->last_request > ros::Duration(3.0))
    {
        mavros_msgs::CommandTOL takeoff_cmd;
        takeoff_cmd.request.altitude = altitude;
        if (_takeoff_client.call(takeoff_cmd))
        {
            ROS_INFO("Vehicle Takeoff to altitude: %0.2f", altitude);
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_INFO("Vehicle Takeoff Failed!");
            this->mission_state = MissionState::arm;
            ROS_INFO("Mission mode -> arm");
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

bool BaseConductor::land(double delay)
{
    if (this->current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(delay)))
    {
        mavros_msgs::CommandTOL land_cmd;
        if (_land_client.call(land_cmd) &&
            land_cmd.response.success)
        {
            ROS_INFO("Vehicle landed!");
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_INFO("Vehicle land failed!");
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}
