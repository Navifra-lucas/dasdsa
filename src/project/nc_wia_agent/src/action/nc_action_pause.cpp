#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_pause.h>
#include <std_msgs/String.h>

using namespace NaviFra;

NcActionPause::NcActionPause()
{
    task_cmd_pub_ = nh.advertise<std_msgs::String>("/nc_task_manager/task_cmd", 10);
}

NcActionPause::~NcActionPause()
{
}

std::string NcActionPause::implName()
{
    return "NcActionPause";
}

void NcActionPause::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Pause command received from RID: %s", rid);

        // 실제 취소 처리 로직 (예시)
        // TaskManager::instance().cancelCurrentTask();
        std_msgs::String cmd_msg;
        cmd_msg.data = "pause";
        task_cmd_pub_.publish(cmd_msg);
        robotStatus->setACSPause(true);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "pause");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Pause Exception: " << ex.displayText();
    }
}
