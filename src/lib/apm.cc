#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/PositionTarget.h>

#include "conductor/ansi_color.h"
#include "conductor/apm.h"

// ArduConductor::ArduConductor(int &argc, char **argv, const std::string &name, double rate, uint32_t options)
// 	: BaseConductor(argc, argv, name, rate, options)
// {
// 	initNode();
// }

ArduConductor::ArduConductor(ros::NodeHandle &nodehandle, double rate)
	: BaseConductor(nodehandle, rate)
{
	initNode();
}

/// @brief 设置全局坐标原点
/// @param latitude 纬度
/// @param longitude 经度
void ArduConductor::sendGpOrigin(double latitude, double longitude)
{
	geographic_msgs::GeoPointStamped gp_origin;
	gp_origin.position.latitude = latitude;
	gp_origin.position.longitude = longitude;
	gp_origin.position.altitude = 0;
	ros::Time ros_time = ros::Time::now();
	gp_origin.header.stamp.sec = ros_time.sec;
	gp_origin.header.stamp.nsec = ros_time.nsec;
	set_gp_origin_pub_.publish(gp_origin);
	ROS_INFO(SUCCESS("Setting gp origin...(%0.2f, %0.2f)"), longitude, latitude);
}

/// @brief 设置移动速度
/// @param speed
/// @return
bool ArduConductor::setMoveSpeed(double speed)
{
	ros::ServiceClient speed_client = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
	mavros_msgs::CommandLong msg;
	msg.request.command = mavros_msgs::CommandCode::DO_CHANGE_SPEED;
	msg.request.param1 = 0; // ignored by Ardupilot
	msg.request.param2 = speed;

	if (speed_client.call(msg) && msg.response.success)
	{
		ROS_INFO(SUCCESS("Speed set: %0.2f"), speed);
		return true;
	}
	else
	{
		ROS_ERROR("Failed set speed");
		return false;
	}
}

/// @brief flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
/// @param x
/// @param y
/// @param z
/// @param yaw_rate
/// @return
void ArduConductor::setSpeedBody(double x, double y, double z, double yaw_rate)
{
	using namespace mavros_msgs;
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_PX |
						   PositionTarget::IGNORE_PY |
						   PositionTarget::IGNORE_PZ |
						   PositionTarget::IGNORE_AFX |
						   PositionTarget::IGNORE_AFY |
						   PositionTarget::IGNORE_AFZ;
	if (fabs(yaw_rate) < 1e-3)
	{
		raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
		raw_target.yaw = 0;
	}
	else
	{
		raw_target.type_mask |= PositionTarget::IGNORE_YAW;
	}

	raw_target.velocity.x = x;
	raw_target.velocity.y = y;
	raw_target.velocity.z = z;
	raw_target.yaw_rate = yaw_rate;
	set_raw_pub_.publish(raw_target);
}

/// @brief FLU rad/s , must set continously, or the vehicle stops after a few seconds.(failsafe feature) used for adjusting yaw without setting others.
/// @param yaw_rate
/// @return
void ArduConductor::setAngularRate(double yaw_rate)
{
	using namespace mavros_msgs;
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX |
						   PositionTarget::IGNORE_VY |
						   PositionTarget::IGNORE_VZ |
						   PositionTarget::IGNORE_AFX |
						   PositionTarget::IGNORE_AFY |
						   PositionTarget::IGNORE_AFZ |
						   PositionTarget::IGNORE_YAW; // yaw_rate must be used with pose or vel.
	raw_target.position.x = 0;
	raw_target.position.y = 0;
	raw_target.position.z = 0;
	raw_target.yaw_rate = yaw_rate;
	set_raw_pub_.publish(raw_target);
}

/// @brief flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
/// @param x
/// @param y
/// @param z
/// @param yaw
/// @return
void ArduConductor::setPoseBody(double x, double y, double z, double yaw)
{
	using namespace mavros_msgs;
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX |
						   PositionTarget::IGNORE_VY |
						   PositionTarget::IGNORE_VZ |
						   PositionTarget::IGNORE_AFX |
						   PositionTarget::IGNORE_AFY |
						   PositionTarget::IGNORE_AFZ |
						   PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = x;
	raw_target.position.y = y;
	raw_target.position.z = z;
	raw_target.yaw = yaw;
	set_raw_pub_.publish(raw_target);
}

void ArduConductor::setPoseWorld(double x, double y, double z, double yaw)
{
	using namespace mavros_msgs;
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_LOCAL_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX |
						   PositionTarget::IGNORE_VY |
						   PositionTarget::IGNORE_VZ |
						   PositionTarget::IGNORE_AFX |
						   PositionTarget::IGNORE_AFY |
						   PositionTarget::IGNORE_AFZ |
						   PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = x;
	raw_target.position.y = y;
	raw_target.position.z = z;
	raw_target.yaw = yaw;
	set_raw_pub_.publish(raw_target);
}

/// @brief 悬停
void ArduConductor::setBreak()
{
	using namespace mavros_msgs;
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX |
						   PositionTarget::IGNORE_VY |
						   PositionTarget::IGNORE_VZ |
						   PositionTarget::IGNORE_AFX |
						   PositionTarget::IGNORE_AFY |
						   PositionTarget::IGNORE_AFZ |
						   PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = 0;
	raw_target.position.y = 0;
	raw_target.position.z = 0;
	raw_target.yaw = 0;
	set_raw_pub_.publish(raw_target);
}

void ArduConductor::initNode()
{
	set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>("mavros/global_position/set_gp_origin", 10);
}