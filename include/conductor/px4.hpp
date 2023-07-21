#ifndef PX4_H
#define PX4_H
#include "conductor/base.h"

class PX4Conductor: public BaseConductor
{
public:
    PX4Conductor(int &argc, char **argv, const std::string &name, double rate = 20, uint32_t options = 0) : BaseConductor(argc, argv, name, rate, options){};
    PX4Conductor(ros::NodeHandle *nodehandle, double rate = 20) : BaseConductor(nodehandle, rate){};

    ~PX4Conductor(){};
};

#endif // PX4_H