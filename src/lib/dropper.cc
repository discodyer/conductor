#include "conductor/dropper.h"

Dropper::Dropper(ros::NodeHandle &nh)
    : nh_(nh), is_allow_takeoff(false), takeoff_mode_(DropperTakeoffMode::kForbid)
{
    this->init();
}

void Dropper::init()
{
    drop_pub_ = nh_.advertise<std_msgs::String>("dropper", 1);
    takeoff_sub_ = nh_.subscribe<std_msgs::String>("dropper/takeoff", 10, &Dropper::subTakeoffCallback, this);
}

Dropper::~Dropper() {}

void Dropper::dropAll()
{
    std_msgs::String msg;
    msg.data = std::string("drop all");
    drop_pub_.publish(msg);
}

void Dropper::pickAll()
{
    std_msgs::String msg;
    msg.data = std::string("pick all");
    drop_pub_.publish(msg);
}

void Dropper::drop(int num)
{
    std_msgs::String msg;
    switch (num)
    {
    case 1:
        msg.data = std::string("drop 1");
        break;

    case 2:
        msg.data = std::string("drop 2");
        break;

    case 3:
        msg.data = std::string("drop 3");
        break;

    default:
        return;
        break;
    }
    drop_pub_.publish(msg);
}

void Dropper::subTakeoffCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string takeoff_cmd = msg->data;
    if (takeoff_cmd == std::string("takeoff normal"))
    {
        takeoff_mode_ = DropperTakeoffMode::kNormal;
    }
    else if (takeoff_cmd == std::string("takeoff fallback"))
    {
        takeoff_mode_ = DropperTakeoffMode::kFallBack;
    }
    else if (takeoff_cmd == std::string("takeoff 1"))
    {
        takeoff_mode_ = DropperTakeoffMode::kTakeoff1;
    }
    else if (takeoff_cmd == std::string("takeoff 2"))
    {
        takeoff_mode_ = DropperTakeoffMode::kTakeoff2;
    }
    else
    {
        takeoff_mode_ = DropperTakeoffMode::kForbid;
    }
    is_allow_takeoff = (takeoff_mode_ != DropperTakeoffMode::kForbid);
}
