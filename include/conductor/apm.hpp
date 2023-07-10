#ifndef APM_HPP
#define APM_HPP
#include "conductor/base.hpp"

class ArduConductor : public BaseConductor
{
public:
    ArduConductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0) : BaseConductor(argc, argv, name, rate, options){};
    ArduConductor(ros::NodeHandle *nodehandle, double rate = 20) : BaseConductor(nodehandle, rate){};

    ~ArduConductor(){};
};

#endif // APM_HPP