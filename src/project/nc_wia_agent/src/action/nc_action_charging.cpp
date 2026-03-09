#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <task_msgs/Charging.h>
#include <nc_wia_agent/action/nc_action_charging.h>

using namespace NaviFra;

NcActionCharging::NcActionCharging()
{
    pub_charging_ = nh_.advertise<task_msgs::Charging>("/nc_task_manager/charging", 10, false);
}

NcActionCharging::~NcActionCharging()
{
}

std::string NcActionCharging::implName()
{
    return "NcActionCharging";
}

void NcActionCharging::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("CHarging command received from RID: %s", rid);
        task_msgs::Charging msg;
        msg.mode = "charging";
        pub_charging_.publish(msg);

        int err_code = 0;
        bool success = true;

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "charging");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("err_code", err_code);
        response->set("success", success);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Charging Exception: " << ex.displayText();
    }
}
