#ifndef DROPPER_H
#define DROPPER_H
#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

class Dropper
{
public:
    Dropper(ros::NodeHandle &nh);
    ~Dropper();
    void dropAll();
    void pickAll();
    void drop(int num);
private:
    ros::NodeHandle nh_;         // ROS 节点句柄
    ros::Publisher drop_pub_;
};

#endif // DROPPER_H