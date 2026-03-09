#ifndef NAVIFRA_MOTOR_COMMANDER_MIDDLEWARE_UPPER_HPP
#define NAVIFRA_MOTOR_COMMANDER_MIDDLEWARE_UPPER_HPP

#include "Upper/interface/ITurntable.hpp"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_msgs/MotorTarget.h"
#include "motor_msgs/MotorTargetInfo.h"
#include "motor_msgs/UpperTurntableCommand.h"
#include "motor_msgs/UpperTurntableData.h"
#
#include "ros/ros.h"

#include <optional>

namespace NaviFra {
namespace MotorCommander {
namespace Upper {

class Turntable : public ITurntable {
public:
    Turntable(float ppr, float gear_ratio)
        : ppr_{ppr}
        , gear_ratio_{gear_ratio}
        , ref_pos_is_init_pos_{true}
    {
    }
    Turntable(float ppr, float gear_ratio, int32_t reference_position)
        : ppr_{ppr}
        , gear_ratio_{gear_ratio}
        , ref_pos_is_init_pos_{false}
        , reference_position_{reference_position}
    {
    }

    bool initialize(void);
    bool finalize(void);
    void Preset(int32_t position) override;

    void MoveAbsoluteByPosition(int32_t position) override;
    void MoveRelativeByPosition(int32_t position) override;
    void MoveAbsoluteByDegree(float degree) override;
    void MoveRelativeByDegree(float degree) override;

    void IsHome(void) override;
    void IsPositive(void) override;
    void IsNegative(void) override;

private:
    const float ppr_;
    const float gear_ratio_;
    const bool ref_pos_is_init_pos_;
    ros::Subscriber sub_motor_data_info_;

    ros::Publisher pub_upper_turntable_info_;
    ros::Subscriber sub_upper_turntable_command_;

    ros::Publisher pub_motor_target_;

    int32_t reference_position_{0};

    std::atomic<int32_t> current_position_{0};

    motor_msgs::MotorTargetInfo cmd_;

    void RecvMotorDataInfo(const motor_msgs::MotorDataInfo::ConstPtr& msg);
    void RecvUpperTurntableCommand(const motor_msgs::UpperTurntableCommand::ConstPtr& msg);
};

}  // namespace Upper
}  // namespace MotorCommander
}  // namespace NaviFra

#endif