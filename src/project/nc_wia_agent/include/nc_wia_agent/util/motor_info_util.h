#ifndef NAVIFRA_MOTOR_INFO_UTIL_H
#define NAVIFRA_MOTOR_INFO_UTIL_H

#include <Poco/JSON/Object.h>
#include <motor_msgs/MotorDataInfo.h>

namespace NaviFra {

Poco::JSON::Object::Ptr toMotorInfoObject(const motor_msgs::MotorDataInfo& info);

}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_INFO_UTIL_H
