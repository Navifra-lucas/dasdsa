#ifndef NAVIFRA_NAVICAN__INTERFACE_MOTOR_STATE_PROCESSOR_H
#define NAVIFRA_NAVICAN__INTERFACE_MOTOR_STATE_PROCESSOR_H

#include <chrono>
#include <cstdint>

namespace NaviFra {
class IMotorStateProcessor {
public:
    virtual ~IMotorStateProcessor() = default;

    virtual bool handleInit() = 0;
    virtual void handleRead() = 0;
    virtual void handleWrite() = 0;
    virtual bool handleEnable() = 0;
    virtual bool handleDisable() = 0;
    virtual bool handleShutdown() = 0;
    virtual bool handleHalt() = 0;
    virtual bool handleRecover() = 0;
    virtual bool handleHoming(int8_t homing_method, std::chrono::milliseconds timeout) = 0;
};
}  // namespace NaviFra

#endif