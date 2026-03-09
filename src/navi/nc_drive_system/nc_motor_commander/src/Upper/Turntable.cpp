#include "Upper/Turntable.hpp"

#include "data_struct.hpp"
#include "util/logger.hpp"

#include <mutex>

using namespace NaviFra::MotorCommander::Upper;

bool Turntable::initialize(void)
{
    sub_motor_data_info_ =
        ros::NodeHandle().subscribe<motor_msgs::MotorDataInfo>("/motor_data/info", 10, &Turntable::RecvMotorDataInfo, this);

    pub_upper_turntable_info_ = ros::NodeHandle().advertise<motor_msgs::UpperTurntableData>("/motor_commander/upper/turntable/info", 10);
    pub_motor_target_ = ros::NodeHandle().advertise<motor_msgs::MotorTargetInfo>("/motor_target", 10);
    sub_upper_turntable_command_ = ros::NodeHandle().subscribe<motor_msgs::UpperTurntableCommand>(
        "/motor_commander/upper/turntable/command", 5, &Turntable::RecvUpperTurntableCommand, this);

    return true;
}

bool Turntable::finalize(void)
{
    return true;
}

void Turntable::RecvMotorDataInfo(const motor_msgs::MotorDataInfo::ConstPtr& msg)
{
    static std::once_flag flag;
    std::call_once(flag, [&]() {
        if (ref_pos_is_init_pos_ == true) {
            auto utt = std::find_if(msg->data.begin(), msg->data.end(), [](const motor_msgs::MotorData& data) {
                return (data.motor_id == MOTOR_UPPER_TURNTABLE);
            });
            if (utt != msg->data.end()) {
                reference_position_ = utt->encoder;
            }
        }
    });

    motor_msgs::UpperTurntableData data;

    auto utt = std::find_if(
        msg->data.begin(), msg->data.end(), [](const motor_msgs::MotorData& data) { return (data.motor_id == MOTOR_UPPER_TURNTABLE); });
    if (utt != msg->data.end()) {
        data.reference_position = reference_position_;
        data.actual_position = utt->encoder;
        data.actual_degree = (utt->encoder - reference_position_) / (ppr_ * gear_ratio_) * 360.0f;
        data.is_home = false;
        data.is_negative = false;
        data.is_positive = false;
        data.is_moving = false;
        pub_upper_turntable_info_.publish(data);
    }
}

void Turntable::Preset(int32_t position)
{
}

void Turntable::RecvUpperTurntableCommand(const motor_msgs::UpperTurntableCommand::ConstPtr& msg)
{
    NLOG(info) << "cmd: " << msg->cmd << ", position: " << msg->position << ", degree: " << msg->degree;
    if (msg->cmd == "move_relative_by_degree") {
        MoveRelativeByDegree(msg->degree);
    }
}
void Turntable::MoveAbsoluteByPosition(int32_t position)
{
    cmd_.data.reserve(1);
    cmd_.data.emplace_back();
    auto& data = cmd_.data.back();

    data.motor_id = MOTOR_UPPER_TURNTABLE;
    data.operation_mode = 1;
    data.target = static_cast<float>(position);
    pub_motor_target_.publish(cmd_);
}

void Turntable::MoveRelativeByPosition(int32_t position)
{
    cmd_.data.reserve(1);
    cmd_.data.emplace_back();
    auto& data = cmd_.data.back();

    data.motor_id = MOTOR_UPPER_TURNTABLE;
    data.operation_mode = 1;
    data.target = static_cast<float>(position + reference_position_);
    pub_motor_target_.publish(cmd_);
}

void Turntable::MoveAbsoluteByDegree(float degree)
{
    float position = degree * ppr_ * gear_ratio_ / 360.0f;
    cmd_.data.reserve(1);
    cmd_.data.emplace_back();
    auto& data = cmd_.data.back();

    data.motor_id = MOTOR_UPPER_TURNTABLE;
    data.operation_mode = 1;
    data.target = static_cast<float>(position);
    pub_motor_target_.publish(cmd_);
}

void Turntable::MoveRelativeByDegree(float degree)
{
    float position = degree * ppr_ * gear_ratio_ / 360.0f;
    cmd_.data.reserve(1);
    cmd_.data.emplace_back();
    auto& data = cmd_.data.back();

    data.motor_id = MOTOR_UPPER_TURNTABLE;
    data.operation_mode = 1;
    data.target = static_cast<float>(position + reference_position_);
    pub_motor_target_.publish(cmd_);
}

void Turntable::IsHome(void)
{
}

void Turntable::IsPositive(void)
{
}

void Turntable::IsNegative(void)
{
}
