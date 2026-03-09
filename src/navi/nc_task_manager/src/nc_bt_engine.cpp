#include "nc_task_manager/nc_task_manager_pch.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <nc_task_manager/nc_bt_engine.h>

using namespace NaviFra;

NcBehaviorTreeEngine::NcBehaviorTreeEngine(BT::Tree& tree, BT::Blackboard& blackboard)
    : tree_(tree)
    , blackboard_(blackboard)
    , isRunning_(false)
    , cancelRequest_(false)
{
    ros::NodeHandle nh;
    task_alarm_ = nh.advertise<core_msgs::TaskAlarm>("nc_task_manager/task_alarm", 1, false);
}

NcBehaviorTreeEngine::~NcBehaviorTreeEngine()
{
    stop();
}

void NcBehaviorTreeEngine::run()
{
    try {
        while (isRunning_) {
            BT::NodeStatus status = tree_.tickOnce();

            if (cancelRequest_) {
                NLOG(info) << "cancelRequest terminate Thread";
                isRunning_ = false;
                cancelRequest_ = false;
                tree_.haltTree();
                break;
            }

            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::IDLE) {
                BT::SharedQueue<Task> tasks;
                if (blackboard_.get("tasks", tasks) && tasks->size() == 0) {
                    isRunning_ = false;
                    NLOG(info) << " Running -> Success or Idle Status ";
                }
                else {
                    auto current_task = tasks->at(0);
                    if (current_task.type() == TYPE.MOVE || current_task.type() == TYPE.UNDOCKING) {
                        blackboard_.set("need_update_goal", true);
                        // core_msgs::TaskAlarm alarm;
                        // alarm.alarm = ALARM.START;
                        // alarm.uuid = current_task.uuid();
                        // alarm.type = current_task.type();
                        // task_alarm_.publish(alarm);
                    }
                    tree_.rootNode()->haltNode();
                }
            }
            Poco::Thread::sleep(10);
        }
        Poco::Thread::sleep(1);
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }

    isRunning_ = false;
    NLOG(info) << "BehaviorTreeEngine is susccess";
}

void NcBehaviorTreeEngine::start()
{
    NLOG(info) << "start";

    Poco::FastMutex::ScopedLock lock(mutex_);
    worker_.setPriority(Poco::Thread::Priority::PRIO_HIGHEST);
    if (isRunning()) {
        NLOG(warning) << "BehaviorTreeEngine is Running";
        return;
    }
    isRunning_ = true;
    cancelRequest_ = false;

    blackboard_.set("need_update_goal", true);
    blackboard_.set("driverDoneReceived", false);
    blackboard_.set("charge_start_received", "false");
    blackboard_.set("charge_stop_received", false);
    blackboard_.set("charge_stop_trigger", false);

    blackboard_.set("checking_start_received", "false");
    blackboard_.set("loading_received", "false");
    blackboard_.set("docking_reached", "false");
    blackboard_.set("turn_reached", "false");
    blackboard_.set("action_reached", "false");
    blackboard_.set("undocking_reached", "false");
    blackboard_.set("uncharge_received", false);
    blackboard_.set("fork_lift_reached", false);
    blackboard_.set("wait_cancel", false);
    blackboard_.set("load_state_received", false);
    blackboard_.set("wingbody_perception_reached", false);

    if (worker_.isRunning() != true) {
        // blackboard_.set("driverDoneReceived", false);
        worker_.start(*this);
    }
    else {
        NLOG(info) << "Is Running Behavior Tree";
    }
    NLOG(info) << "BehaviorTreeEngine is started";
}

void NcBehaviorTreeEngine::stop()
{
    Poco::FastMutex::ScopedLock lock(mutex_);
    try {
        if (isRunning()) {
            NLOG(info) << "Stopping BehaviorTreeEngine...";
            isRunning_ = false;

            tree_.haltTree();
            worker_.join();

            NLOG(info) << "Stop success BehaviorTreeEngine";
        }
        else {
            NLOG(warning) << "BehaviorTreeEngine is not running";
        }
    }
    catch (Poco::TimeoutException& ex) {
        NLOG(error) << "BehaviorTreeEngine stop Timeout";
    }
}

void NcBehaviorTreeEngine::cancel()
{
    try {
        Poco::FastMutex::ScopedLock lock(mutex_);
        cancelRequest_ = true;
        worker_.join();
        blackboard_.set("tasks", std::make_shared<std::deque<Task>>());
        blackboard_.set("driverDoneReceived", false);
        blackboard_.set("charge_start_received", "false");
        blackboard_.set("charge_stop_received", false);
        blackboard_.set("charge_stop_trigger", false);
        blackboard_.set("checking_start_received", "false");
        blackboard_.set("loading_received", "false");
        blackboard_.set("unloading_received", "false");
        blackboard_.set("docking_reached", "false");
        blackboard_.set("turn_reached", "false");
        blackboard_.set("action_reached", "false");
        blackboard_.set("undocking_reached", "false");
        blackboard_.set("fork_lift_reached", false);
        blackboard_.set("load_state_received", false);
        blackboard_.set("wingbody_perception_reached", false);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void NcBehaviorTreeEngine::pause()
{
    stop();
}

bool NcBehaviorTreeEngine::isRunning()
{
    return isRunning_;
}