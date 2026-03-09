#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <Poco/DateTimeFormatter.h>
#include <Poco/JSON/Object.h>
#include <Poco/UUIDGenerator.h>
#include <core_agent/message/message_broker.h>
#include <nc_wia_agent/data/status_channel.h>
#include <nc_wia_agent/util/plc_data_publisher.h>

#include <iostream>  // 로그용
#include <sstream>

namespace NaviFra {

PlcTaskPublisher& PlcTaskPublisher::instance()
{
    static PlcTaskPublisher inst;
    return inst;
}

PlcTaskPublisher::PlcTaskPublisher() = default;
PlcTaskPublisher::~PlcTaskPublisher() = default;

void PlcTaskPublisher::controlScenario(bool b_action_type, int n_scenario_index)
{
    std::lock_guard<std::mutex> lock(mtx_);

    try {
        Poco::JSON::Object::Ptr msg = new Poco::JSON::Object;
        msg->set("id", Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
        msg->set("time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y.%m.%d %H:%M:%S.%i"));
        msg->set("subject", "WRITE_PLC_DATA");

        Poco::JSON::Object::Ptr payload = new Poco::JSON::Object;
        payload->set("id", Poco::Dynamic::Var());
        if (n_scenario_index == 1) {
            payload->set("tag", "SCENARIO_1.0");
        }
        else if (n_scenario_index == 2) {
            payload->set("tag", "SCENARIO_2.0");
        }
        else if (n_scenario_index == 3) {
            payload->set("tag", "SCENARIO_3.0");
        }
        else if (n_scenario_index == 4) {
            payload->set("tag", "SCENARIO_4.0");
        }
        else {
            NLOG(error) << "[PLC] Invalid scenario index: " << n_scenario_index;
            return;
        }
        payload->set("val", b_action_type);
        payload->set("producer", "H_ACS");
        payload->set("plc", "LoadUnload");

        msg->set("payload", payload);

        std::ostringstream oss;
        msg->stringify(oss);
        MessageBroker::instance().publish("PLC-SC_LoadUnload.H_ACS", oss.str());
        NLOG(info) << "[PLC] Published Scenario Control: " << oss.str();
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "controlScenario Exception: " << ex.displayText();
    }
}

void PlcTaskPublisher::publishOffset(std::string name, double value)
{
    std::lock_guard<std::mutex> lock(mtx_);

    try {
        Poco::JSON::Object::Ptr msg = new Poco::JSON::Object;
        msg->set("id", Poco::UUIDGenerator::defaultGenerator().createRandom().toString());
        msg->set("time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y.%m.%d %H:%M:%S.%i"));
        msg->set("subject", "WRITE_PLC_DATA");

        Poco::JSON::Object::Ptr payload = new Poco::JSON::Object;
        payload->set("id", Poco::Dynamic::Var());
        payload->set("tag", name);
        payload->set("val", std::to_string(value));
        payload->set("producer", "H_ACS");
        payload->set("plc", "OFFSET");

        msg->set("payload", payload);

        std::ostringstream oss;
        msg->stringify(oss);
        MessageBroker::instance().publish("PLC-SC_OFFSET.H_ACS", oss.str());
        NLOG(info) << "[VISION] Published OFFSET: " << oss.str();
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "publishOffset Exception: " << ex.displayText();
    }
}

}  // namespace NaviFra
