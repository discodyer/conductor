#ifndef APM_HPP
#define APM_HPP
#include "conductor/base.hpp"

class ArduConductor : public BaseConductor
{
protected:
    ros::Publisher _set_gp_origin_pub; // 发布器 - 设置地理坐标原点
    void init_node() override;

public:
    ArduConductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0);
    ArduConductor(ros::NodeHandle *nodehandle, double rate = 20);
    
    void send_gp_origin(double latitude = 32.108693377508494, double longitude = 118.92943049870283);
    bool set_move_speed(double speed);
    void set_speed_body(double x, double y, double z, double yaw_rate); 
    void set_angular_rate(double yaw_rate);
    void set_pose_body(double x, double y, double z, double yaw);
    void set_break();

    ~ArduConductor(){};
};

#endif // APM_HPP