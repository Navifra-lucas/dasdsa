#ifndef NAVIFRA_CUSTOM_DRIVER_PARAM_MSG_H_
#define NAVIFRA_CUSTOM_DRIVER_PARAM_MSG_H_

#include "interface/data_struct.hpp"
#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>

#include "core/util/logger.hpp"

namespace NaviFra {
class DriverParam {
public:
    DriverParam(){};
    virtual ~DriverParam(){};

    bool b_robot_init_ = false;

    void UpdateParam(
        ros::NodeHandle& nh, ros::NodeHandle& nhp, InterfaceParameters_t& st_interface_param_, DriverParameters_t& st_driver_param_);
};
}  // namespace NaviFra
#endif
