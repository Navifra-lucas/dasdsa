#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/manager/publish_manager.h>
#include <nc_wia_agent/util/task_result_publisher.h>
#include <nc_wia_agent/data/status_channel.h>

using namespace NaviFra;

#include <Poco/DateTimeFormat.h>
#include <Poco/DateTimeFormatter.h>
#include <Poco/JSON/Object.h>
#include <ros/time.h>

#include <sstream>

int TaskResultPublisher::getResultValue(std::string state)
{
    int value;

    if (state == "PENDING") {
        value = 0;
    }
    else if (state == "ACTIVE") {
        value = 1;
    }
    else if (state == "PREEMPTED") {
        value = 2;
    }
    else if (state == "SUCCEEDED") {
        value = 3;
    }
    else if (state == "ABORTED") {
        value = 4;
    }
    else if (state == "REJECTED") {
        value = 5;
    }
    else if (state == "PREEMPTING") {
        value = 6;
    }
    else if (state == "RECALLING") {
        value = 7;
    }
    else if (state == "RECALLED") {
        value = 8;
    }
    else if (state == "LOST") {
        value = 9;
    }

    return value;
};

namespace NaviFra {

TaskResultPublisher& TaskResultPublisher::instance()
{
    static TaskResultPublisher instance;
    return instance;
}

TaskResultPublisher::TaskResultPublisher()
    : msg_id_(0)
{
}
TaskResultPublisher::~TaskResultPublisher()
{
}

void TaskResultPublisher::setTaskContext(const std::string& rid, const std::string& task_id, int msg_id)
{
    std::lock_guard<std::mutex> lock(mtx_);
    rid_ = rid;
    task_id_ = task_id;
    msg_id_ = msg_id;
    NLOG(info) << "Task context set: RID=" << rid_ << ", TaskID=" << task_id_;
}

void TaskResultPublisher::publish(std::string result)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (rid_.empty() || task_id_.empty()) {
        // auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        // rid_ = robotStatus->getRobotID();
        // task_id_ = robotStatus->getTaskID();
        // robotStatus->clearTaskID();
        // msg_id_ = robotStatus->getMsgID();
        NLOG(error) << "Task context is not set.";
        return;
    }

    try {
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        Poco::JSON::Object::Ptr stamp = new Poco::JSON::Object;
        ros::Time now = ros::Time::now();
        stamp->set("secs", now.sec);
        stamp->set("nsecs", now.nsec);

        Poco::JSON::Object::Ptr header = new Poco::JSON::Object;
        header->set("seq", 0);
        header->set("stamp", stamp);
        header->set("frame_id", "");

        Poco::JSON::Object::Ptr status_obj = new Poco::JSON::Object;
        Poco::JSON::Object::Ptr goal_id = new Poco::JSON::Object;
        goal_id->set("stamp", stamp);
        goal_id->set("id", task_id_);
        status_obj->set("goal_id", goal_id);

        int n_status = getResultValue(result);
        std::string s_result = (n_status == 3) ? "S" : "F";
        status_obj->set("status", getResultValue(result));
        status_obj->set("text", result);

        Poco::JSON::Object::Ptr result2 = new Poco::JSON::Object;
        result2->set("elapsed_time", 0.0);

        response->set("header", header);
        response->set("status", status_obj);
        response->set("result", result2);
        response->set("Cmd", "result");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id_);
        response->set("RID", rid_);
        response->set("Result", s_result);
        response->set("AmrId", rid_);

        std::ostringstream oss;
        response->stringify(oss);
        MessageBroker::instance().publish(rid_ + ".ACS", oss.str());
        NLOG(info) << "Published Task Result Message: " << oss.str();
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Task Result Exception: " << ex.displayText();
    }
}

}  // namespace NaviFra