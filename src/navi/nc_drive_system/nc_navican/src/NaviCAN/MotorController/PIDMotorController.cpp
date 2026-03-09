#include "NaviCAN/MotorController/PIDMotorController.h"

namespace NaviFra {

void PIDMotorController::setTargetPosition(double target_position) {
    target_position_ = target_position;
}

void PIDMotorController::start()
{
    running_ = true;
    control_thread_ = std::thread(&PIDMotorController::run, this);
}

void PIDMotorController::stop()
{
    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

void PIDMotorController::update(double dt)
{
    // 요기 뭘 수정 해야 될 것 같은데 그게 뭘까요?
    double current_position = getPosition();
    double error = target_position_ - current_position;
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;
    setTarget(control_signal);
}

void PIDMotorController::run()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    while (running_) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = current_time - start_time;
        double dt = elapsed_time.count();

        update(dt);

        start_time = current_time;
        std::this_thread::sleep_for(update_interval_);
    }
}

}