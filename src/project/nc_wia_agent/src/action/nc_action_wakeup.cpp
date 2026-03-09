#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <core_msgs/HacsWakeup.h>
#include <nc_wia_agent/action/nc_action_wakeup.h>

using namespace NaviFra;

NcActionWakeup::NcActionWakeup()
{
    wakeup_pub_ = nh_.advertise<core_msgs::HacsWakeup>("/nc_task_manager/wake_up", 10);
}

NcActionWakeup::~NcActionWakeup()
{
}

std::string NcActionWakeup::implName()
{
    return "NcActionWakeup";
}

void NcActionWakeup::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");
        std::string wakeup_id = obj->getValue<std::string>("data");

        NLOG(info) << Poco::format("Wakeup command received from RID: %s", rid);

        core_msgs::HacsWakeup msg;
        msg.wake_up = true;
        msg.id = wakeup_id;
        wakeup_pub_.publish(msg);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "wakeup");
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
        NLOG(error) << "Wakeup Exception: " << ex.displayText();
    }
}
