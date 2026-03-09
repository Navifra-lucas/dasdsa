#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_battery.h>
#include <nc_wia_agent/data/battery_status.h>

using namespace NaviFra;

NcActionBattery::NcActionBattery()
{
}

NcActionBattery::~NcActionBattery()
{
}

std::string NcActionBattery::implName()
{
    return "NcActionBattery";
}

void NcActionBattery::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");
            
        float soc =InMemoryRepository::instance().get<BatteryStatus>(BatteryStatus::KEY)->getCapacity();
        NLOG(info) << Poco::format("Battery command received from RID: %s", rid);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "battery");
        response->set("data", InMemoryRepository::instance().get<BatteryStatus>(BatteryStatus::KEY)->toArray());
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
        NLOG(info) << "Published Task Result Message: " << oss.str();

    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Battery Exception: " << ex.displayText();
    }
}
