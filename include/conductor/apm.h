#ifndef APM_H
#define APM_H
#include "conductor/base.h"
#include "conductor/waypoint.h"
#include <tf2_ros/transform_broadcaster.h>

// 起飞状态
enum class APMTakeoffState
{
    kLand,
    kArmed,
    kTakeoff1,
    kTakeoff2,
};

class ArduConductor : public BaseConductor
{
protected:
    ros::Publisher set_gp_origin_pub_; // 发布器 - 设置地理坐标原点
    void initNode() override;
    APMTakeoffState flag_takeoff;

public:
    // ArduConductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0);
    ArduConductor(ros::NodeHandle &nodehandle, double rate = 20);

    bool takeoff(double pre_takeoff_alt, double altitude, double delay = 3.0); // 重写二段起飞方法
    bool arm(double delay = 5.0);
    bool land(double delay = 10.0);

    void sendGpOrigin(double latitude = 32.108693377508494, double longitude = 118.92943049870283) const;
    bool setMoveSpeed(double speed);
    void setSpeedBody(double x, double y, double z, double yaw_rate);
    void setAngularRate(double yaw_rate);
    void setPoseBody(double x, double y, double z, double yaw);
    void setPoseRelated(double x, double y, double z, double yaw);
    void setPoseWorld(double x, double y, double z, double yaw) const;
    void setWaypointPoseWorld(waypoint::Waypoint waypoint);
    void sendTranslatedPoseWorld(double x, double y, double z, double yaw) const;
    void setBreak();

    ~ArduConductor(){};
};

#endif // APM_H