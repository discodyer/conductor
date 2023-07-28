#include "conductor/fixed_point.h"
#include "conductor/ansi_color.h"

FixedPoint::FixedPoint(const std::string &topic, fixed_point::Point center, ros::NodeHandle &nh, const PidParams &params_x, const PidParams &params_y)
    : center_(center), nh_(nh), pid_controller_x(params_x.kp, params_x.ki, params_x.kd, params_x.windup_guard, params_x.output_bound, params_x.sample_time),
      pid_controller_y(params_y.kp, params_y.ki, params_y.kd, params_y.windup_guard, params_y.output_bound, params_y.sample_time), output_bound_(params_x.output_bound, params_y.output_bound)
{
    point_sub_ = nh_.subscribe<geometry_msgs::Point>(topic, 10, &FixedPoint::subPointCallback, this);
}

void FixedPoint::subPointCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // Opencv 坐标系 往右X增大 往下Y增大
    // 飞机坐标系 X对应Opencv的-Y Y对应Opencv的-X
    ROS_INFO(SUCCESS("\n--------------------------"));
    this->point_ = *msg;
    ROS_INFO(SUCCESS("\nGot point \nX: %0.2f\nY: %0.2f"), point_.x, point_.y);
    auto offset = fixed_point::Point(point_.y - center_.y , point_.x - center_.x);
    ROS_INFO(SUCCESS("\nOffset \nX: %0.2f\nY: %0.2f"), offset.x, offset.y);
    calcXYOutput(offset);
    ROS_INFO(SUCCESS("\nOutput \nX: %0.2f\nY: %0.2f"), last_output_.x, last_output_.y);
    ROS_INFO(SUCCESS("\n--------------------------"));
}

auto FixedPoint::calcXYOutput(fixed_point::Point offset) -> fixed_point::Point
{
    last_output_.x = pid_controller_x.calcOutput(offset.x);
    last_output_.y = pid_controller_y.calcOutput(offset.y);
    return last_output_;
}

auto FixedPoint::calcXYOutput() -> fixed_point::Point
{
    return last_output_;
}

auto FixedPoint::getBoundedOutput(fixed_point::Point offset) -> fixed_point::Point
{
    calcXYOutput(offset);
    return {Bound<double>(last_output_.x, output_bound_.x), Bound<double>(last_output_.y, output_bound_.y)};
}

auto FixedPoint::getBoundedOutput() -> fixed_point::Point
{
    return {Bound<double>(last_output_.x, output_bound_.x), Bound<double>(last_output_.y, output_bound_.y)};
}

void FixedPoint::clear()
{
    last_output_.x = 0.0;
    last_output_.y = 0.0;
    pid_controller_x.clear();
    pid_controller_y.clear();
}