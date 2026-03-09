#include "nc_wia_agent/util/motor_info_util.h"

#include <Poco/JSON/Array.h>

namespace NaviFra {

namespace {
Poco::JSON::Object::Ptr makeFloatArrayFrom(
    const motor_msgs::MotorDataInfo& info, std::function<float(const motor_msgs::MotorData&)> extractor)
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
    Poco::JSON::Array::Ptr arr = new Poco::JSON::Array;
    for (const auto& m : info.data)
        arr->add(extractor(m));
    obj->set("data", arr);
    return obj;
}
}  // namespace

Poco::JSON::Object::Ptr toMotorInfoObject(const motor_msgs::MotorDataInfo& info)
{
    Poco::JSON::Object::Ptr motor_info = new Poco::JSON::Object;

    motor_info->set("voltage", makeFloatArrayFrom(info, [](const auto& m) { return m.voltage; }));
    motor_info->set(
        "temperature", makeFloatArrayFrom(info, [](const auto& m) { return m.status; }));  // 예시로 status 사용 (온도 필드 없으면)
    motor_info->set("current", makeFloatArrayFrom(info, [](const auto& m) { return m.current; }));
    motor_info->set("fault", makeFloatArrayFrom(info, [](const auto& m) { return m.is_error ? 1.0f : 0.0f; }));

    return motor_info;
}

}  // namespace NaviFra
