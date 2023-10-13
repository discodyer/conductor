#include "conductor/fixed_point.h"
#include "conductor/ansi_color.h"
#include <std_msgs/String.h>

FixedPoint::FixedPoint(const std::string &topic, fixed_point::Point center, ros::NodeHandle &nh, const PidParams &params_x, const PidParams &params_y)
    : center_(center), nh_(nh), pid_controller_x(params_x.kp, params_x.ki, params_x.kd, params_x.windup_guard, params_x.output_bound, params_x.sample_time),
      pid_controller_y(params_y.kp, params_y.ki, params_y.kd, params_y.windup_guard, params_y.output_bound, params_y.sample_time), output_bound_(params_x.output_bound, params_y.output_bound)
{
    if (topic != "none")
    {
        point_sub_ = nh_.subscribe<geometry_msgs::Point>(topic, 10, &FixedPoint::subPointCallback, this);
    }
}

void FixedPoint::subPointCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // Opencv 坐标系 往右X增大 往下Y增大
    // 飞机坐标系 X对应Opencv的-Y Y对应Opencv的-X
    ROS_INFO(SUCCESS("\n--------------------------"));
    this->point_ = *msg;
    ROS_INFO(SUCCESS("\nGot point \nX: %0.2f\nY: %0.2f"), point_.x, point_.y);
    auto offset = fixed_point::Point(point_.y - center_.y, point_.x - center_.x);
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

FixedPointYolo::FixedPointYolo(const std::string &topic, const std::string &yolo_tag,
                               fixed_point::Point center, double lock_distance, ros::NodeHandle &nh,
                               const PidParams &params_x, const PidParams &params_y, int lock_cutoff)
    : FixedPoint("none", center, nh, params_x, params_y), yolo_tag_(yolo_tag), is_found_(false),
      lock_distance_(lock_distance), locked_count_(0), is_dropped_(false), is_locked_(false), lock_cutoff_(lock_cutoff),
      last_frame_time_(ros::Time::now()), timeout_count_(0)
{
    beep_pub_ = nh_.advertise<std_msgs::String>("dropper/beep", 1);
    point_yolo_sub_ = nh_.subscribe<conductor::yolo>(topic, 10, &FixedPointYolo::subPointYoloCallback, this);
}

bool FixedPointYolo::is_lock_counter_reached()
{
    return locked_count_ > lock_cutoff_;
}

void FixedPointYolo::subPointYoloCallback(const conductor::yolo::ConstPtr &msg)
{
    // Opencv 坐标系 往右X增大 往下Y增大
    // 飞机坐标系 X对应Opencv的-Y Y对应Opencv的-X
    this->point_yolo_ = *msg;
    if (point_yolo_.class_name == yolo_tag_)
    {
        this->is_found_ = true;
        this->updateTime();
        // this->beep();
        // ROS_INFO(SUCCESS("\n--------------------------"));
        // ROS_INFO(SUCCESS("\nGot a Yolo tag: %s"), point_yolo_.class_name.c_str());
        // ROS_INFO(SUCCESS("\npoint \nX: %0.2f\nY: %0.2f"), point_yolo_.center_x, point_yolo_.center_y);
        auto offset = fixed_point::Point(point_yolo_.center_y - center_.y, point_yolo_.center_x - center_.x);
        double distance_ = std::sqrt(offset.x * offset.x + offset.y * offset.y);
        if (distance_ < lock_distance_)
        {
            locked_count_++;
            // ROS_INFO(SUCCESS("\nOffset \nX: %0.2f\nY: %0.2f, distance: %0.2f"), offset.x, offset.y, distance_);
        }
        else
        {
            locked_count_ = 0;
            // ROS_INFO(COLORED_TEXT("\nOffset \nX: %0.2f\nY: %0.2f, distance: %0.2f", ANSI_BACKGROUND_RED), offset.x, offset.y, distance_);
        }
        calcXYOutput(offset);
        // ROS_INFO(SUCCESS("\nOutput \nX: %0.2f\nY: %0.2f"), last_output_.x, last_output_.y);
        // ROS_INFO(SUCCESS("\n--------------------------"));
    }
}

void FixedPointYolo::beep()
{
    std_msgs::String msg;
    beep_pub_.publish(msg);
}

void FixedPointYolo::reset()
{
    is_found_ = false;
    locked_count_ = 0;
    is_dropped_ = false;
    is_locked_ = false;
}

void FixedPointYolo::updateTime()
{
    this->last_frame_time_ = ros::Time::now();
}

bool FixedPointYolo::isTimeout()
{
    return (ros::Time::now() - this->last_frame_time_) > ros::Duration(1.0);
}