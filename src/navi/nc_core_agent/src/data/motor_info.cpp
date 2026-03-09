#include "core_agent/core_agent.h"

#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>
#include <core_agent/data/motor_info.h>

#include <regex>

using Poco::JSON::Object;

using namespace NaviFra;

const std::string MotorInfoStore::KEY = "MotorInfoStore";

CMotorInfo::CMotorInfo()
{
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;

    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = 0.0;
    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = 0.0;
    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = 0.0;
    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = 0.0;
}

CMotorInfo::CMotorInfo(core_msgs::MotorInfo::ConstPtr msg)
{
    Poco::Timestamp now;

    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_FL_traction_motor_current;
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_FL_traction_motor_target_rpm;
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_FL_traction_motor_feedback_rpm;
    motors_[MOTOR::MOTOR_TRACTION_FL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_FL_traction_motor_encoder;
    timeStamps_[MOTOR::MOTOR_TRACTION_FL] = now;

    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_FR_traction_motor_current;
    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_FR_traction_motor_target_rpm;
    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_FR_traction_motor_feedback_rpm;
    // motors_[MOTOR::MOTOR_TRACTION_FR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_FR_traction_motor_encoder;
    // timeStamps_[MOTOR::MOTOR_TRACTION_FR] = now;

    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_RL_traction_motor_current;
    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_RL_traction_motor_target_rpm;
    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_RL_traction_motor_feedback_rpm;
    // motors_[MOTOR::MOTOR_TRACTION_RL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_RL_traction_motor_encoder;
    // timeStamps_[MOTOR::MOTOR_TRACTION_RL] = now;

    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_RR_traction_motor_current;
    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_RR_traction_motor_target_rpm;
    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_RR_traction_motor_feedback_rpm;
    motors_[MOTOR::MOTOR_TRACTION_RR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_RR_traction_motor_encoder;
    timeStamps_[MOTOR::MOTOR_TRACTION_RR] = now;

    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_FL_steer_motor_current;
    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_FL_steer_motor_target_rpm;
    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_FL_steer_motor_feedback_rpm;
    motors_[MOTOR::MOTOR_STEER_FL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_FL_steer_absencoder_feedback_deg;  // 일단 임시
    timeStamps_[MOTOR::MOTOR_STEER_FL] = now;

    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_FR_steer_motor_current;
    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_FR_steer_motor_target_rpm;
    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_FR_steer_motor_feedback_rpm;
    // motors_[MOTOR::MOTOR_STEER_FR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_FR_steer_absencoder_feedback_deg;  // 일단 임시
    // timeStamps_[MOTOR::MOTOR_STEER_FR] = now;

    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_RL_steer_motor_current;
    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_RL_steer_motor_target_rpm;
    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_RL_steer_motor_feedback_rpm;
    // motors_[MOTOR::MOTOR_STEER_RL].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_RL_steer_absencoder_feedback_deg;  // 일단 임시
    // timeStamps_[MOTOR::MOTOR_STEER_RL] = now;

    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_CURRENT) = msg->f_RR_steer_motor_current;
    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_TARGET_RPM) = msg->f_RR_steer_motor_target_rpm;
    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_FEEDBACK_RPM) = msg->f_RR_steer_motor_feedback_rpm;
    motors_[MOTOR::MOTOR_STEER_RR].at(MOTOR_INFO::MOTOR_INFO_ENCODER) = msg->f_RR_steer_absencoder_feedback_deg;  // 일단 임시
    timeStamps_[MOTOR::MOTOR_STEER_RR] = now;
}

CMotorInfo::~CMotorInfo()
{
    motors_.clear();
}

MotorInfoStore::MotorInfoStore()
{
    motor_name_[MOTOR_TRACTION_FL] = "fl_traction";
    // motor_name_[MOTOR_TRACTION_FR] = "fr_traction";
    // motor_name_[MOTOR_TRACTION_RL] = "rl_traction";
    motor_name_[MOTOR_TRACTION_RR] = "rr_traction";
    motor_name_[MOTOR_STEER_FL] = "fl_steer";
    // motor_name_[MOTOR_STEER_FR] = "fr_steer";
    // motor_name_[MOTOR_STEER_RL] = "rl_steer";
    motor_name_[MOTOR_STEER_RR] = "rr_steer";
}

void MotorInfoStore::append(core_msgs::MotorInfo::ConstPtr msg)
{
    motor_infos_.emplace_back(CMotorInfo(msg));
}

std::string MotorInfoStore::toString(std::string id)
{
    Object obj, motors;

    obj.set("id", id);

    for (size_t motor = 0; motor < MOTOR::MOTOR_LENGTH; motor++) {
        motors.set(motor_name_.at(motor), aggregateMotorInfo(MOTOR(motor)));
    }

    obj.set("motors", motors);
    std::ostringstream oss;
    obj.stringify(oss);

    return oss.str();
}

Poco::JSON::Object::Ptr MotorInfoStore::toObject()
{
    Object motors;
    Object::Ptr obj = new Object();

    for (size_t motor = 0; motor < MOTOR::MOTOR_LENGTH; motor++) {
        motors.set(motor_name_.at(motor), aggregateMotorInfo(MOTOR(motor)));
    }

    obj->set("motors", motors);

    return std::move(obj);
}

Poco::JSON::Object MotorInfoStore::aggregateMotorInfo(MOTOR motor)
{
    Poco::JSON::Object obj;
    Poco::JSON::Array currents, targets, feedbacks, timestamps;
    for (auto info : motor_infos_) {
        auto motors = info.getMotors();
        auto timeStamp = info.getTimeStamps();
        currents.add(motors[motor].at(MOTOR_INFO_CURRENT));
        targets.add(motors[motor].at(MOTOR_INFO_TARGET_RPM));
        feedbacks.add(motors[motor].at(MOTOR_INFO_FEEDBACK_RPM));
        timestamps.add(timeStamp.at(motor).raw() / 1000);
    }

    obj.set("targets", targets);
    obj.set("feedbacks", feedbacks);
    obj.set("currents", currents);
    obj.set("timestamps", timestamps);

    return obj;
}

void MotorInfoStore::clear()
{
    motor_infos_.clear();
}