#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include "tf2_ros/transform_broadcaster.h"
#include <std_msgs/String.h>
#include <libserial/SerialStream.h>
#include <cmath>
#include "conductor/ansi_color.h"

bool ready_flag = false;
bool fire_flag = false;
geometry_msgs::Point fire_point;

void readyCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == std::string("ready"))
    {
        ready_flag = true;
        ROS_INFO("ready");
    }
}

void fireCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    fire_flag = true;
    fire_point = *msg;
    ROS_INFO("got a fire point: x: %f, y: %f", fire_point.x, fire_point.y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_com");
    ros::NodeHandle nh;

    ros::Publisher drone_takeoff_pub_ = nh.advertise<std_msgs::String>("drone/takeoff", 100);
    ros::Subscriber drone_ready_sub_ = nh.subscribe("drone/ready", 100, readyCallback);
    ros::Subscriber drone_fire_sub_ = nh.subscribe("drone/fire", 100, fireCallback);
    // ros::Subscriber drone_takeoff_sub_ = nh.subscribe("drone/pose", 100, poseCallback);

    std::string serial_port_name_ = "/dev/ttyS0";
    LibSerial::SerialStream serial_stream_;

    try
    {
        serial_stream_.Open(serial_port_name_);
        serial_stream_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_stream_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_stream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_stream_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_stream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        ROS_INFO(SUCCESS("Serial port %s opened!"), serial_port_name_.c_str());
    }
    catch (const LibSerial::OpenFailed &e)
    {
        ROS_ERROR("Serial port open failed : %s", e.what());
        return 0;
    }

    ros::Rate rate(20);
    while (ros::ok())
    {
        const int BUFFER_SIZE = 128;
        char input_buffer[BUFFER_SIZE] = {};
        int input_len = 0;
        while (serial_stream_.IsDataAvailable())
        {
            serial_stream_ >> input_buffer[input_len++];
        }

        switch (input_buffer[1])
        {
        case 0xaa: // 起飞
            if (input_buffer[2] == 0x6b)
            {
                std_msgs::String msg_;
                msg_.data = std::string("takeoff_common");
                ROS_INFO("takeoff_common");
                drone_takeoff_pub_.publish(msg_);
            }
            break;
        
        case 0xab: // 起飞 提高
            if (input_buffer[2] == 0x6b)
            {
                std_msgs::String msg_;
                msg_.data = std::string("takeoff_harder");
                ROS_INFO("takeoff_harder");
                drone_takeoff_pub_.publish(msg_);
            }
            break;
        default:
            break;
        }

        if(ready_flag)
        {
            serial_stream_ << (uint8_t)0xac << (uint8_t)0x01 << (uint8_t)0x6b; 
            ready_flag = false;
        }

        if(fire_flag)
        {
            char output_buffer[9];
            output_buffer[0] = (uint8_t)0xac;
            output_buffer[1] = (uint8_t)0x03;
            output_buffer[8] = (uint8_t)0x6b;
            output_buffer[2] = fire_point.x > 0 ? 0 : 1;
            int fire_x_int = static_cast<int>(fire_point.x * 1000);
            output_buffer[3] = (fire_x_int >> 8) & 0xff;
            output_buffer[4] = fire_x_int & 0xff;
            output_buffer[5] = fire_point.y > 0 ? 0 : 1;
            int fire_y_int = static_cast<int>(fire_point.y * 1000);
            output_buffer[6] = (fire_y_int >> 8) & 0xff;
            output_buffer[7] = fire_y_int & 0xff;
            serial_stream_.write(output_buffer, 9);

            fire_flag = false;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}