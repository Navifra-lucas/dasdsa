#ifndef NC_BT_ENGINE_H
#define NC_BT_ENGINE_H

#include <Poco/Mutex.h>
#include <Poco/Runnable.h>
#include <Poco/Thread.h>

#include "core_msgs/GoalList.h"
#include <core_msgs/TaskAlarm.h>

namespace NaviFra {
class NcBehaviorTreeEngine : public Poco::Runnable {
public:
    NcBehaviorTreeEngine(BT::Tree& tree, BT::Blackboard& blackboard);
    ~NcBehaviorTreeEngine();

    using Ptr = std::shared_ptr<NcBehaviorTreeEngine>;

public:
    void start();
    void stop();
    bool isRunning();
    void cancel();
    void pause();

private:
    virtual void run();

private:
    Poco::FastMutex mutex_;
    BT::Tree& tree_;
    BT::Blackboard& blackboard_;
    std::shared_ptr<BT::Groot2Publisher> groot2_;

    Poco::Thread worker_;
    bool isRunning_;
    bool cancelRequest_;
    ros::Publisher task_alarm_;
};
}  // namespace NaviFra
#endif