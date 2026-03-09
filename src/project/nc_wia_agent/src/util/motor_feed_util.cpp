#include "nc_wia_agent/util/motor_feed_util.h"

namespace NaviFra {

Poco::JSON::Object::Ptr toTotalMotorFeedObject(
    const motor_msgs::MotorDataInfo& drivingMotors, float lift_rpm, float lift_current, float turntable_rpm, float turntable_current)
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;

    // rpm
    Poco::JSON::Object::Ptr rpm = new Poco::JSON::Object;
    Poco::JSON::Array::Ptr driving_rpm = new Poco::JSON::Array;
    for (const auto& m : drivingMotors.data) {
        driving_rpm->add(m.input_velocity);
    }

    rpm->set("driving", driving_rpm);
    rpm->set("lift", lift_rpm);
    rpm->set("turntable", turntable_rpm);
    obj->set("rpm", rpm);

    // current
    Poco::JSON::Object::Ptr current = new Poco::JSON::Object;
    Poco::JSON::Array::Ptr driving_current = new Poco::JSON::Array;
    for (const auto& m : drivingMotors.data) {
        driving_current->add(m.current);
    }
    current->set("driving", driving_current);
    current->set("lift", lift_current);
    current->set("turntable", turntable_current);
    obj->set("current", current);

    return obj;
}

}  // namespace NaviFra
