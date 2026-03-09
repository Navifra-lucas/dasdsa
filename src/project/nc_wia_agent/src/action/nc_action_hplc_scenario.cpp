#include <nc_wia_agent/action/nc_action_hplc_scenario.h>

using namespace NaviFra;

NcActionPlcScenario::NcActionPlcScenario()
{
}
NcActionPlcScenario::~NcActionPlcScenario()
{
}

std::string NcActionPlcScenario::implName()
{
    return "NcActionPlcScenario";
}

void NcActionPlcScenario::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        if (!obj || !obj->has("plcdatas"))
            return;

        Poco::JSON::Array::Ptr plcdatas = obj->getArray("plcdatas");
        if (!plcdatas)
            return;

        bool b_scenario_1 = false;
        bool b_scenario_2 = false;
        bool b_scenario_3 = false;
        bool b_scenario_4 = false;

        for (size_t i = 0; i < plcdatas->size(); ++i) {
            Poco::JSON::Object::Ptr plc = plcdatas->getObject(i);
            if (!plc || !plc->has("data"))
                continue;

            Poco::JSON::Array::Ptr data_arr = plc->getArray("data");
            if (!data_arr)
                continue;

            for (size_t j = 0; j < data_arr->size(); ++j) {
                Poco::JSON::Object::Ptr item = data_arr->getObject(j);
                if (!item || !item->has("tag") || !item->has("val"))
                    continue;

                std::string tag = item->getValue<std::string>("tag");

                if (tag.find("SCENARIO_1.0") == 0) {
                    bool val = item->getValue<bool>("val");
                    b_scenario_1 = val;
                    continue;
                }

                if (tag.find("SCENARIO_2.0") == 0) {
                    bool val = item->getValue<bool>("val");
                    b_scenario_2 = val;
                    continue;
                }

                if (tag.find("SCENARIO_3.0") == 0) {
                    bool val = item->getValue<bool>("val");
                    b_scenario_3 = val;
                    continue;
                }

                if (tag.find("SCENARIO_4.0") == 0) {
                    bool val = item->getValue<bool>("val");
                    b_scenario_4 = val;
                    continue;
                }
            }

            bool b_scenario_1_active = status->isScenario1Active();
            bool b_scenario_2_active = status->isScenario2Active();

            if (b_scenario_1 != b_scenario_1_active && b_scenario_2 != b_scenario_1_active) {
                status->Scenario1Active(b_scenario_1);
                NLOG(info) << "[PLC] Scenario 1 Activated";
            }
            if (b_scenario_3 != b_scenario_2_active && b_scenario_4 != b_scenario_2_active) {
                status->Scenario2Active(b_scenario_3);
                NLOG(info) << "[PLC] Scenario 2 Activated";
            }
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << "[PLC] Exception: " << e.what();
    }
}
