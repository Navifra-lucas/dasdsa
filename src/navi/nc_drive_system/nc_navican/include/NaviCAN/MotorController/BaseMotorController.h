#ifndef NAVIFRA_NAVICAN_BASE_MOTOR_CONTROLLER_H
#define NAVIFRA_NAVICAN_BASE_MOTOR_CONTROLLER_H

#include "NaviCAN/EncoderDriver/interface/IExternalEncoder.h"
#include "NaviCAN/MotorController/interface/IMotorController.h"
#include "NaviCAN/MotorDriver/interface/IMotorInfo.h"
#include "NaviCAN/MotorDriver/interface/IMotorStateProcessor.h"
#include "NaviCAN/MotorExtraInfo/interface/IMotorExtraInfo.h"

namespace NaviFra {

class BaseMotorController : public IMotorController {
public:
    BaseMotorController(
        std::shared_ptr<IMotorInfo> info, std::shared_ptr<IMotorStateProcessor> processor, std::shared_ptr<IExternalEncoder> encoder,
        std::shared_ptr<IMotorExtraInfo> extra_info)
        : info_(info)
        , processor_(processor)
        , encoder_(encoder)
        , extra_info_(extra_info)
    {
    }

    bool init() { return processor_->handleInit(); }
    bool enable() override { return processor_->handleEnable(); }
    bool disable() override { return processor_->handleDisable(); }
    bool shutdown() override { return processor_->handleShutdown(); }
    bool resetError() override { return processor_->handleRecover(); }
    bool homing(int8_t homing_method, std::chrono::milliseconds timeout) override { return processor_->handleHoming(homing_method, timeout); }

    int getState() override { return info_->getState(); }
    std::string getStateText() override { return info_->getStateText(); }
    bool isEnable() override { return info_->isEnable(); }
    bool isFault() override { return info_->isFault(); }
    uint16_t getStatus() override { return info_->getStatus(); }

    void setTarget(double target) override
    {
        target_ = target;
        info_->setTargetInfo(target_);
    }
    double getTarget() override { return target_; }
    int8_t getOperationMode() override { return info_->getOperationMode(); }
    double getSpeed() override { return info_->getSpeed(); }

    double getPosition() override { return (encoder_ == nullptr) ? info_->getPosition() : encoder_->getPosition(); }
    void presetEncoder() override { (encoder_ == nullptr) ? info_->preset() : encoder_->preset(); }

    double getVoltage() override { return extra_info_->getVoltage(); }
    double getCurrent() override { return extra_info_->getCurrent(); }
    uint32_t getErrorCode() override { return extra_info_->getErrorCode(); }
    std::string getErrorMessage() override { return extra_info_->getErrorMessage(); }
    uint16_t getSTOCode() override { return extra_info_->getSTOCode(); }

private:
    std::shared_ptr<IMotorInfo> info_;
    std::shared_ptr<IMotorStateProcessor> processor_;
    std::shared_ptr<IExternalEncoder> encoder_;
    std::shared_ptr<IMotorExtraInfo> extra_info_;

    double target_;
};
}  // namespace NaviFra

#endif  // NAVIFRA_PIDCONTROLLER_H