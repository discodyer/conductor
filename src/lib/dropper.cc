#include "conductor/dropper.h"

Dropper::Dropper(ros::NodeHandle &nh)
    : nh_(nh)
{
    drop_pub_ = nh.advertise<std_msgs::String>("dropper", 1);
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
