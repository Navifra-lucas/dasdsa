#ifndef NAVIFRA_MOTOR_FEED_UTIL_H
#define NAVIFRA_MOTOR_FEED_UTIL_H

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <motor_msgs/MotorDataInfo.h>

#include <vector>

namespace NaviFra {

/**
 * driving 모터들 + lift/turntable 값을 JSON 구조로 변환
 */
Poco::JSON::Object::Ptr toTotalMotorFeedObject(
    const motor_msgs::MotorDataInfo& drivingMotors, float lift_rpm = 0.0f, float lift_current = 0.0f, float turntable_rpm = 0.0f,
    float turntable_current = 0.0f);

}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_FEED_UTIL_H
