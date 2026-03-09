#include "nc_brain_agent/nc_brain_agent.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <Poco/Mutex.h>
#include <Poco/ScopedLock.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/service/nc_repeat_drive_service.h>
#include <nc_brain_agent/utils/nc_agent_config.h>

using namespace NaviFra;

NcRepeatDriveService::NcRepeatDriveService()
{
}

NcRepeatDriveService::~NcRepeatDriveService()
{
}

void NcRepeatDriveService::initialize(ros::NodeHandle& nh)
{
    start_flag = false;
    rosSubscriber_.push_back(nh.subscribe("/repeat_test_log", 1, &NcRepeatDriveService::onRepeatLog, this));
    rosSubscriber_.push_back(nh.subscribe("/repeat/cmd", 1, &NcRepeatDriveService::onRepeatStatus, this));
}

void NcRepeatDriveService::run()
{
    while (isRunning_) {
        std::string info(Poco::format(
            "{\"id\":\"%s\",\"status\":\"%s\",\"log\":\"%s\"}", Config::instance().getString("robot_id", "5000"), repeatstatus_,
            repeatlog_));

        MessageBroker::instance().publish(
            NcBrainMessage::MESSAGE_ROBOT_STATUS_REPEAT_DRIVE + Config::instance().getString("robot_id", "5000"), info);

        if (repeatstatus_ == "completed" && start_flag) {
            start_flag = false;
            stop();
        }

        Poco::Thread::sleep(100);
    };
}

void NcRepeatDriveService::onRepeatLog(const std_msgs::String msg)
{
    repeatlog_ = msg.data;
}

void NcRepeatDriveService::onRepeatStatus(const std_msgs::String msg)
{
    if (msg.data == "running")
        start_flag = true;

    if (msg.data == "idle")
        repeatstatus_ = "completed";
    else
        repeatstatus_ = msg.data;
}