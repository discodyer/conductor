#ifndef APM_H
#define APM_H
#include "conductor/base.h"

class ArduConductor : public BaseConductor
{
protected:
    ros::Publisher set_gp_origin_pub_; // 发布器 - 设置地理坐标原点
    void initNode() override;

public:
    ArduConductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0);
    ArduConductor(ros::NodeHandle *nodehandle, double rate = 20);
    
    void sendGpOrigin(double latitude = 32.108693377508494, double longitude = 118.92943049870283);
    bool setMoveSpeed(double speed);
    void setSpeedBody(double x, double y, double z, double yaw_rate); 
    void setAngularRate(double yaw_rate);
    void setPoseBody(double x, double y, double z, double yaw);
    void setBreak();

    ~ArduConductor(){};
};

#endif // APM_H