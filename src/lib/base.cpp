#include "conductor/base.hpp"
#include "conductor/ansi_color.hpp"

/// @brief
/// @param argc
/// @param argv
/// @param name 节点名称
/// @param rate_num loop循环频率
/// @param options 节点实例化选项
BaseConductor::BaseConductor(int &argc, char **argv, const std::string &name, double rate_num, uint32_t options) : rate(rate_num)
{
    ros::init(argc, argv, name, options);
    mission_state = prearm;
    this->init_node();
}

/// @brief
/// @param nodehandle 直接传入外部 nodehandle
/// @param rate_num loop循环频率
BaseConductor::BaseConductor(ros::NodeHandle *nodehandle, double rate_num) : rate(rate_num), _nh(*nodehandle)
{
    mission_state = prearm;
    this->init_node();
}

/// @brief 初始化各种订阅和发布服务
void BaseConductor::init_node()
{
    setlocale(LC_ALL, "");
    ROS_INFO(COLORED_TEXT("Initializing node...", "\033[1m" ANSI_COLOR_MAGENTA));
    _state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, &BaseConductor::sub_state_cb, this);
    _local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    _set_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ROS_INFO(COLORED_TEXT("高性能ですから!", "\033[2m" ANSI_COLOR_WHITE));
}

/// @brief mavros/state 订阅回调函数
/// @param msg
void BaseConductor::sub_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    this->current_state = *msg;
}

/// @brief 设置飞行模式为 guided
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::set_mode_guided(double delay)
{
    ros::ServiceClient _set_mode_client; // 客户端 - 设置模式
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mavros_msgs::SetMode mode_guided_msg;
    mode_guided_msg.request.custom_mode = "GUIDED";
    if (is_time_elapsed(delay))
    {
        if (_set_mode_client.call(mode_guided_msg) &&
            mode_guided_msg.response.mode_sent)
        {
            ROS_INFO(SUCCESS("Guided enabled!"));
            this->mission_state = MissionState::arm;
            ROS_INFO(MISSION_SWITCH_TO("arm"));
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_ERROR("Guided Switch Failed!");
            this->mission_state = MissionState::prearm;
            ROS_INFO(MISSION_SWITCH_TO("prearm"));
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

/// @brief 解锁电机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::arm(double delay)
{
    if (!current_state.armed &&
        is_time_elapsed(delay))
    {
        ros::ServiceClient _arming_client; // 客户端 - 解锁
        _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (_arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
            ROS_INFO(SUCCESS("Vehicle armed!"));
            this->mission_state = MissionState::takeoff;
            ROS_INFO(MISSION_SWITCH_TO("takeoff"));
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_ERROR("Vehicle arm failed!");
            this->mission_state = MissionState::prearm;
            ROS_INFO(MISSION_SWITCH_TO("prearm"));
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

/// @brief 起飞到指定高度 ArduCopter
/// @param altitude 高度 单位 M
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::takeoff(double altitude, double delay)
{
    if (is_time_elapsed(delay))
    {
        ros::ServiceClient _takeoff_client; // 客户端 - 起飞
        _takeoff_client = _nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

        mavros_msgs::CommandTOL takeoff_cmd;
        takeoff_cmd.request.altitude = altitude;
        if (_takeoff_client.call(takeoff_cmd))
        {
            ROS_INFO(SUCCESS("Vehicle Takeoff to altitude: %0.2f"), altitude);
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_ERROR("Vehicle Takeoff Failed!");
            this->mission_state = MissionState::arm;
            ROS_INFO(MISSION_SWITCH_TO("arm"));
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

/// @brief 切换到landed模式（ArduCopter）降落飞机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::land(double delay)
{
    if (this->current_state.armed &&
        is_time_elapsed(delay))
    {
        ros::ServiceClient _land_client; // 客户端 - 降落
        _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

        mavros_msgs::CommandTOL land_cmd;
        if (_land_client.call(land_cmd) &&
            land_cmd.response.success)
        {
            ROS_INFO(SUCCESS("Vehicle landed!"));
            last_request = ros::Time::now();
            return true;
        }
        else
        {
            ROS_ERROR("Vehicle land failed!");
            last_request = ros::Time::now();
            return false;
        }
    }
    return false;
}

/// @brief 判断是否到延迟时间
/// @param delay 延迟时间
/// @return bool
bool BaseConductor::is_time_elapsed(double delay)
{
    if (ros::Time::now() - this->last_request > ros::Duration(delay))
    {
        return true;
    }
    else
    {
        return false;
    }
}