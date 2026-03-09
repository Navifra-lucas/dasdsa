#include "nc_wia_agent/util/wingbody_offset_checker.h"

#include "util/logger.hpp"

using namespace NaviFra;

WingbodyOffsetChecker::WingbodyOffsetChecker()
    : f_received_x_(0.0f)
    , f_received_y_(0.0f)
    , f_received_theta_(0.0f)
    , b_has_data_(false)
    , last_publish_time_()
{
}

WingbodyOffsetChecker::~WingbodyOffsetChecker()
{
}

bool WingbodyOffsetChecker::isNeedCompare() const
{
    return b_has_data_;
}

bool WingbodyOffsetChecker::compare()
{
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

    if (!robotStatus) {
        NLOG(error) << "[WingbodyOffsetChecker] RobotBasicStatus not available!";
        return false;
    }

    std::vector<float> vec_mqtt_offset = robotStatus->getWingbodyOffset();
    NLOG(info) << vec_mqtt_offset[0] << ", " << vec_mqtt_offset[1] << ", " << vec_mqtt_offset[2];

    if (fabs(vec_mqtt_offset[0] - f_received_x_) < 0.001f && fabs(vec_mqtt_offset[1] - f_received_y_) < 0.001f &&
        fabs(vec_mqtt_offset[2] - f_received_theta_) < 0.0002f) {
        NLOG(info) << "MQTT offset is Updated!!!!!";
        return true;
    }
    return false;
}

void WingbodyOffsetChecker::reset()
{
    b_has_data_ = false;
    f_received_x_ = 0.0f;
    f_received_y_ = 0.0f;
    f_received_theta_ = 0.0f;
    NLOG(info) << "[WingbodyOffsetChecker] Data reset";
}

void WingbodyOffsetChecker::update(float x, float y, float theta)
{
    last_publish_time_.update();
    f_received_x_ = x;
    f_received_y_ = y;
    f_received_theta_ = theta;
    b_has_data_ = true;
    NLOG(info) << "[WingbodyOffsetChecker] Try to update hplc offset" << x << ", " << y << ", " << theta;
}

bool WingbodyOffsetChecker::isTimeout() const
{
    if (!b_has_data_) {
        return false;
    }

    Poco::Timestamp now;
    int64_t elapsed_ms = (now - last_publish_time_) / 1000;  // microseconds to milliseconds

    return elapsed_ms > TIMEOUT_MS;
}

void WingbodyOffsetChecker::getData(float& x, float& y, float& theta) const
{
    x = f_received_x_;
    y = f_received_y_;
    theta = f_received_theta_;
}
