#ifndef NAVIFRA_NAVICAN_BASE_MOTOR_DRIVER_H
#define NAVIFRA_NAVICAN_BASE_MOTOR_DRIVER_H

#include "NaviCAN/MotorDriver/interface/IMotorStateProcessor.h"
#include "NaviCAN/NaviCANDriver.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

namespace NaviFra {

// 기본 모터 파라메터
struct BaseMotorDriverParams {
    std::shared_ptr<NaviFra::NaviCANDriver> driver;
    std::chrono::milliseconds handle_interval;
    uint8_t motor_id;

    virtual ~BaseMotorDriverParams() = default;
};

class BaseMotorDriver
    : public NaviFra::IMotorStateProcessor
    , public std::enable_shared_from_this<BaseMotorDriver> {
public:
    BaseMotorDriver(const BaseMotorDriverParams& params)
        : driver_(params.driver)
        , handle_interval_(params.handle_interval)
        , motor_id_(params.motor_id)
    {
    }
    virtual ~BaseMotorDriver() = default;

    // IMotorStateHandler 인터페이스 함수
    bool handleInit() override { return true; }
    void handleRead() override {}
    void handleWrite() override {}
    bool handleEnable() override { return true; }
    bool handleDisable() override { return true; }
    bool handleShutdown() override { return true; }
    bool handleHalt() override { return true; }
    bool handleRecover() override { return true; }
    bool handleHoming(int8_t homing_method, std::chrono::milliseconds timeout) override { return true; }

    template <typename T>
    T getValue(uint16_t index, uint8_t subIndex = 0) const
    {
        return driver_->universal_get_value<T>(index, subIndex);
    }

    template <typename T>
    void setValue(uint16_t index, uint8_t subIndex, T value)
    {
        driver_->universal_set_value<T>(index, subIndex, value);
    }

    template <typename T>
    T getRpdoMapped(uint16_t index, uint8_t subIndex = 0) const
    {
        return static_cast<T>(driver_->rpdo_mapped[index][subIndex]);
    }

    template <typename T>
    void setTpdoMapped(uint16_t index, uint8_t subIndex, T value)
    {
        driver_->tpdo_mapped[index][subIndex] = value;
    }

protected:
    std::shared_ptr<NaviCANDriver> driver_;
    std::chrono::milliseconds handle_interval_;
    const uint8_t motor_id_;

    void startThread();
    void stopThread();

private:
    std::atomic<bool> running_handler_ = false;
    std::thread handler_thread_;

    void handleDrive();
};

}  // namespace NaviFra

#endif
