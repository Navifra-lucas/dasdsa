#ifndef NAVIFRA_NAVICAN_PID_MOTOR_CONTROLLER_H
#define NAVIFRA_NAVICAN_PID_MOTOR_CONTROLLER_H

#include "NaviCAN/MotorController/BaseMotorController.h"

#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

namespace NaviFra {

class PIDMotorController : public NaviFra::BaseMotorController {
public:
    PIDMotorController(double kp, double ki, double kd,
            std::shared_ptr<IMotorInfo> info,
            std::shared_ptr<IMotorStateProcessor> processor,
            std::shared_ptr<IExternalEncoder> encoder,
            std::shared_ptr<IMotorExtraInfo> extra_info) :
        NaviFra::BaseMotorController(info, processor, encoder, extra_info),
        kp_(kp),
        ki_(ki),
        kd_(kd) {}

    ~PIDMotorController() {
        stop();
    }
        
    void setTargetPosition(double target_position);
private:
    double kp_;
    double ki_;
    double kd_;

    void update(double dt);
    void run();
    void start();
    void stop();

    double target_ = 0;
    double target_position_;
    double prev_error_ = 0;;
    double integral_ = 0;

    std::thread control_thread_;
    std::atomic<bool> running_ = false;
    std::chrono::milliseconds update_interval_ = 10ms;
};
}  // namespace NaviFra

#endif  // NAVIFRA_PIDCONTROLLER_H