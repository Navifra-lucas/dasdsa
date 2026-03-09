#ifndef NAVIFRA_DRIVER_PARAM_MSG_H_
#define NAVIFRA_DRIVER_PARAM_MSG_H_

#include "data_struct.hpp"
#include "ros/ros.h"

#include <string>

namespace NaviFra {
class DriverParam {
public:
    void UpdateParam(
        ros::NodeHandle& nh, ros::NodeHandle& nhp, InterfaceParameters_t& st_interface_param_, DriverParameters_t& st_driver_param_);
};
}  // namespace NaviFra
#endif
