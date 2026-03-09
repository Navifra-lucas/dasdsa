#ifndef NAVIFRA_ZMQ_INITIALIZER_H
#define NAVIFRA_ZMQ_INITIALIZER_H

#pragma once

#include "core_agent/manager/initializer_manager.h"

#include <Poco/RunnableAdapter.h>
#include <Poco/ThreadPool.h>

#include <atomic>

namespace NaviFra {

class ZMQInitializer : public Initializer {
public:
    virtual ~ZMQInitializer();
    virtual void finalize() override;
    void initialize() override;
    void shutdown();  ///< 안전 종료 처리
    int priority() const override { return 30; }  // RobotVerify 이후 실행되도록 우선순위 설정

private:
    void pollerLoop();
    std::atomic<bool> running_{false};
};

REGISTER_INITIALIZER(ZMQInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ZMQ_INITIALIZER_H
