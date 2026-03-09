#ifndef NAVIFRA_MOTOR_COMMANDER_MIDDLEWARE_INTERFACE_TURNTABLE_HPP
#define NAVIFRA_MOTOR_COMMANDER_MIDDLEWARE_INTERFACE_TURNTABLE_HPP

#include <cstdint>

namespace NaviFra {
namespace MotorCommander {
namespace Upper {

class ITurntable {
public:
    ~ITurntable() = default;
    virtual void Preset(int32_t position) = 0;

    virtual void MoveAbsoluteByPosition(int32_t position) = 0;
    virtual void MoveRelativeByPosition(int32_t position) = 0;
    virtual void MoveAbsoluteByDegree(float degree) = 0;
    virtual void MoveRelativeByDegree(float degree) = 0;

    virtual void IsHome(void) = 0;
    virtual void IsPositive(void) = 0;
    virtual void IsNegative(void) = 0;
};

}  // namespace Upper
}  // namespace MotorCommander
}  // namespace NaviFra

#endif