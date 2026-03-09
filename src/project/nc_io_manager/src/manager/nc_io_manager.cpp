#include "nc_io_manager/manager/nc_io_manager.h"

#include "util/logger.hpp"

namespace NaviFra {

IOManager::IOManager()
    : initialized_(false)
{
    LOG_INFO("IOManager constructor called");
}

IOManager::~IOManager()
{
    shutdown();
    LOG_INFO("IOManager destructor called");
}

void IOManager::initialize()
{
    if (initialized_) {
        LOG_WARNING("IOManager already initialized");
        return;
    }

    LOG_INFO("Initializing IOManager...");

    // IOController 생성 및 초기화
    ros::NodeHandle nh;
    controller_ = std::make_shared<IOController>(nh);
    controller_->initialize();

    // ROS 인터페이스 설정
    setupROSInterface();

    // IOController 스레드 시작
    controller_->run();

    initialized_ = true;
    LOG_INFO("IOManager initialization completed");
}

void IOManager::setupROSInterface()
{
    LOG_INFO("ROS interface setup completed");
}

void IOManager::shutdown()
{
    if (!initialized_) {
        return;
    }

    LOG_INFO("Shutting down IOManager...");

    // 타이머 정지
    if (status_timer_.isValid()) {
        status_timer_.stop();
    }

    // IOController 정지
    if (controller_) {
        controller_->stop();
    }

    initialized_ = false;
    LOG_INFO("IOManager shutdown completed");
}

}  // namespace NaviFra