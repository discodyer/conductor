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
	: BaseConductor(nodehandle, rate), flag_takeoff(APMTakeoffState::kLand)
{
	initNode();
}

/// @brief 设置全局坐标原点
/// @param latitude 纬度
/// @param longitude 经度
void ArduConductor::sendGpOrigin(double latitude, double longitude) const
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

/// @brief 悬停
void ArduConductor::setBreak()
{
	setSpeedBody(0.0, 0.0, 0.0, 0.0);
}

void ArduConductor::initNode()
{
	set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>("mavros/global_position/set_gp_origin", 10);
}

void ArduConductor::setPoseRelated(double x, double y, double z, double yaw)
{
}

void ArduConductor::setPoseWorld(double x, double y, double z, double yaw) const
{
	sendTranslatedPoseWorld(x, y, z, yaw);
}

void ArduConductor::setPoseWorld(waypoint::Waypoint waypoint)
{
	setWaypointPoseWorld(waypoint);
}

void ArduConductor::setWaypointPoseWorld(waypoint::Waypoint waypoint)
{
	this->setMoveSpeed(waypoint.air_speed);
	this->setPoseWorld(waypoint.position.x,
					   waypoint.position.y,
					   waypoint.position.z,
					   waypoint.yaw);
}

void ArduConductor::sendTranslatedPoseWorld(double x, double y, double z, double yaw) const
{
	using namespace mavros_msgs;
	double gamma_world = -1.5707963;
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX |
						   PositionTarget::IGNORE_VY |
						   PositionTarget::IGNORE_VZ |
						   PositionTarget::IGNORE_AFX |
						   PositionTarget::IGNORE_AFY |
						   PositionTarget::IGNORE_AFZ |
						   PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = -y;
	raw_target.position.y = x;
	raw_target.position.z = z;
	raw_target.yaw = yaw - gamma_world;
	set_raw_pub_.publish(raw_target);
}

/// @brief 解锁电机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool ArduConductor::arm(double delay)
{
	if (!current_state.armed &&
		isTimeElapsed(delay))
	{
		ros::ServiceClient arming_client; // 客户端 - 解锁
		arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

		mavros_msgs::CommandBool arm_cmd;
		arm_cmd.request.value = true;
		if (arming_client.call(arm_cmd) &&
			arm_cmd.response.success)
		{
			ROS_INFO(SUCCESS("Vehicle armed!"));
			this->mission_state = MissionState::kTakeoff;
			this->flag_takeoff = APMTakeoffState::kArmed;
			ROS_INFO(MISSION_SWITCH_TO("takeoff"));
			updateLastRequestTime();
			return true;
		}
		else
		{
			ROS_ERROR("Vehicle arm failed!");
			this->mission_state = MissionState::kPrearm;
			this->flag_takeoff = APMTakeoffState::kLand;
			ROS_INFO(MISSION_SWITCH_TO("prearm"));
			updateLastRequestTime();
			return false;
		}
	}
	return false;
}

/// @brief 切换到landed模式（ArduCopter）降落飞机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool ArduConductor::land(double delay)
{
	if (this->current_state.armed &&
		isTimeElapsed(delay))
	{
		ros::ServiceClient land_client; // 客户端 - 降落
		land_client = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

		mavros_msgs::CommandTOL land_cmd;
		if (land_client.call(land_cmd) &&
			land_cmd.response.success)
		{
			ROS_INFO(SUCCESS("Vehicle landed!"));
			this->flag_takeoff = APMTakeoffState::kLand;
			updateLastRequestTime();
			return true;
		}
		else
		{
			ROS_ERROR("Vehicle land failed!");
			updateLastRequestTime();
			return false;
		}
	}
	return false;
}

/// @brief 起飞到指定高度 ArduCopter
/// @param pre_takeoff_alt 一段起飞高度 单位 M
/// @param altitude 最终起飞高度 单位 M
/// @param delay 距离上一个操作的延迟
/// @return bool
bool ArduConductor::takeoff(double pre_takeoff_alt, double altitude, double delay)
{
	if (isTimeElapsed(delay) && flag_takeoff == APMTakeoffState::kArmed)
	{
		flag_takeoff = APMTakeoffState::kTakeoff1;
		return false;
	}

	if (isTimeElapsed(delay) && flag_takeoff == APMTakeoffState::kTakeoff1)
	{
		ros::ServiceClient takeoff_client; // 客户端 - 起飞
		takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

		mavros_msgs::CommandTOL takeoff_cmd;
		takeoff_cmd.request.altitude = pre_takeoff_alt;
		if (takeoff_client.call(takeoff_cmd))
		{
			ROS_INFO(SUCCESS("Vehicle pre-Takeoff to altitude: %0.2f"), pre_takeoff_alt);
			// updateLastRequestTime();
			this->flag_takeoff = APMTakeoffState::kTakeoff2;
			return false;
		}
		else
		{
			ROS_ERROR("Vehicle pre-Takeoff Failed!");
			this->mission_state = MissionState::kArm;
			ROS_INFO(MISSION_SWITCH_TO("arm"));
			updateLastRequestTime();
			return false;
		}
	}

	if (isTimeElapsed(delay + 2.0) && flag_takeoff == APMTakeoffState::kTakeoff2)
	{
		this->setMoveSpeed(0.2); // 设置空速
		this->setPoseWorld(0,
						   0,
						   altitude,
						   0);

		ROS_INFO(SUCCESS("Vehicle Takeoff to world_frame altitude: %0.2f"), altitude);
		updateLastRequestTime();
		this->flag_takeoff = APMTakeoffState::kArmed;
		return true;
	}

	return false;
}