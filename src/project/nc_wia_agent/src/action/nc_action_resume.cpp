#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_resume.h>
#include <std_msgs/String.h>

using namespace NaviFra;

NcActionResume::NcActionResume()
{
    task_cmd_pub_ = nh.advertise<std_msgs::String>("/nc_task_manager/task_cmd", 10);
}

NcActionResume::~NcActionResume()
{
}

std::string NcActionResume::implName()
{
    return "NcActionResume";
}

void NcActionResume::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Resume command received from RID: %s", rid);

        std_msgs::String cmd_msg;
        cmd_msg.data = "resume";
        task_cmd_pub_.publish(cmd_msg);
        robotStatus->setACSPause(false);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "resume");
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
        NLOG(error) << "Resume Exception: " << ex.displayText();
    }
}
