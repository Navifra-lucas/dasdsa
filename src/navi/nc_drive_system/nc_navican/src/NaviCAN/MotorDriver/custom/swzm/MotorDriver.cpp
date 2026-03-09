#include "NaviCAN/MotorDriver/custom/swzm/MotorDriver.h"

#include "NaviCAN/MotorDriver/MotorDriverRegistrar.h"

namespace NaviFra {
namespace NaviCAN {
namespace MotorDriver {
namespace Custom {
namespace swzm {

bool MotorDriver::setTargetInfo(double target)
{
    double abs_target = fabs(target);

    if (is_target_)
        is_target_ = abs_target > 5.0;
    else
        is_target_ = abs_target > 2.0;

    target_ = is_target_ ? target : 0.0;

    return true;
}

bool MotorDriver::handleInit()
{
    DriveEnable(true);
    std::this_thread::sleep_for(500ms);
    startThread();
    NLOG(info) << "Motor ID: " << static_cast<int>(motor_id_) << " initialized.";
    return true;
}

void MotorDriver::handleRead()
{
    std::bitset<8> status_bits;
    status_bits[0] = PowerStageActive();
    status_bits[1] = Fault();
    status_bits[2] = TorqueMode();
    status_bits[3] = AnalogCan();
    status_bits[4] = CwCanEnable();
    status_bits[5] = Ccw();
    status_bits[6] = SafeStopActive();
    status_bits[7] = 0;  // Toggle bit, NOT USED
    uint8_t status_byte = static_cast<uint8_t>(status_bits.to_ulong());

    if (status_byte_.getValue() != status_byte) {
        NLOG(info) << "Motor ID: " << static_cast<int>(motor_id_) << std::hex << " From: " << (int)status_byte_.getValue()
                   << " To: " << (int)status_byte;
        status_byte_.setValue(status_byte);
    }
}

void MotorDriver::handleWrite()
{
    bool hasError = false;
    std::string errorMsg = "Motor ID: " + std::to_string(static_cast<int>(motor_id_));

    // 에러 체크
    if (status_byte_.isFault()) {
        errorMsg += " [Fault detected]";
        ResetFault(true);
        hasError = true;
    }

    if (status_byte_.isSafeStopActive()) {
        errorMsg += " [Safe Stop Active]";
        hasError = true;
    }

    // 에러 : 1초 후 리턴
    if (hasError) {
        NLOG(info) << errorMsg;
        std::this_thread::sleep_for(1s);
        return;
    }

    // 정상 동작
    if (is_target_)
        BrakeRelease(true);
    else
        BrakeRelease(false);

    SetSpeed(static_cast<int16_t>(target_));
}

bool MotorDriver::handleShutdown()
{
    DriveEnable(false);
    std::this_thread::sleep_for(DRIVE_DISABLE_DELAY);
    return true;
}

bool MotorDriver::handleHalt()
{
    return true;
}

bool MotorDriver::handleRecover()
{
    // DriveEnable(true) 에서 Safe Stop 이 리셋되지 않음.
    DriveEnable(false);
    std::this_thread::sleep_for(DRIVE_DISABLE_DELAY);
    return true;
}

}  // namespace swzm
}  // namespace Custom
}  // namespace MotorDriver
}  // namespace NaviCAN
}  // namespace NaviFra
