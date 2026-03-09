#include "NaviCAN/MotorDriver/BaseMotorDriver.h"
#include "util/logger.hpp"

namespace NaviFra 
{

    void BaseMotorDriver::handleDrive()
    {
        handleRead();
        handleWrite();
    }
    
    void BaseMotorDriver::startThread()
    {
        if (running_handler_.load()) {
            NLOG(severity_level::error) << "[MotorId " << int(motor_id_) <<"] Handler thread is already running.";
            return;
        }
    
        running_handler_ = true;
        
        auto self = shared_from_this();
        handler_thread_ = std::thread([self]() {
            NLOG(severity_level::info) << "[MotorId " << int(self->motor_id_) <<"] loop started.";
            try {
                while (self->running_handler_.load()) {
                    self->handleDrive();
                    std::this_thread::sleep_for(self->handle_interval_);
                }
            }
            catch(std::exception& e) {
                NLOG(info) << "[MotorId " << int(self->motor_id_) <<"] e=" << e.what();
            }

            NLOG(severity_level::info) << "[MotorId " << int(self->motor_id_) 
                                       << "] loop ended.";
        });
    }

    void BaseMotorDriver::stopThread()
    {
        if (!running_handler_.load()) {
            NLOG(severity_level::error) << "[MotorId " << int(motor_id_) <<"] Handler thread is not running.";
            return;
        }
    
        running_handler_ = false;
        if (handler_thread_.joinable()) {
            handler_thread_.join();
        }
        NLOG(severity_level::info) <<  "[MotorId " << int(motor_id_) <<"] Handler thread stopped.";
    }

    
} // namespace NaviFra {
