#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_amr_info.h>
#include <nc_wia_agent/data/robot_motor_feed_store.h>
#include <nc_wia_agent/util/motor_feed_util.h>
#include <nc_wia_agent/util/motor_info_util.h>

using namespace NaviFra;

NcActionAMRinfo::NcActionAMRinfo()
{
}

NcActionAMRinfo::~NcActionAMRinfo()
{
}

std::string NcActionAMRinfo::implName()
{
    return "NcActionAMRinfo";
}

void NcActionAMRinfo::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        // std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;

        // 기본 응답 필드
        response->set("Cmd", "amr_info");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        // 예시 데이터
        Poco::JSON::Array::Ptr mileage = new Poco::JSON::Array;
        mileage->add(3.240578575748083);
        mileage->add(7.198200225830078);
        mileage->add(210706.0);
        response->set("mileage", mileage);

        Poco::JSON::Array::Ptr phm_unit_count = new Poco::JSON::Array;
        for (int val : {0, 0, 0, 2, 0})
            phm_unit_count->add(val);
        response->set("PHM_unit_count", phm_unit_count);

        auto store = InMemoryRepository::instance().get<RobotMotorFeedStore>(RobotMotorFeedStore::KEY);
        if (store) {
            const auto& driving = store->getDrivingMotors();
            response->set(
                "total_motor_feed",
                toTotalMotorFeedObject(
                    driving,
                    0,  // turntable rpm
                    0,  // lift rpm
                    0,  // turntable current
                    0  // lift current
                    ));
            response->set("motor_info", NaviFra::toMotorInfoObject(driving));
        }

        // phm
        // Poco::JSON::Array::Ptr phm_array = new Poco::JSON::Array;
        // std::vector<std::tuple<std::string, std::string, std::string>> phm_data = {
        //     {"PHM_OutputVoltage", "float", "54"},
        //     {"PHM_CurrentActual", "float", "36"},
        //     {"PHM_TorqueActual", "float", "0.0"},
        //     {"PHM_TemperatureActual", "float", "29.98046875"},
        //     {"PHM_SpeedActual", "int", "0.0"},
        //     {"PHM_Velocity_Setpoint", "float", "100.0"},
        //     {"PHM_Deceleration_Setpoint", "float", "3000.0"},
        //     {"PHM_State_Stopper", "bool", "False"},
        //     {"PHM_State_PowerMoller", "bool", "False"},
        //     {"PHM_State_DriveBrake", "bool", "False"},
        // };
        // for (auto& [name, type, data] : phm_data) {
        //     Poco::JSON::Object::Ptr phm_entry = new Poco::JSON::Object;
        //     phm_entry->set("name", name);
        //     phm_entry->set("type", type);
        //     phm_entry->set("data", data);
        //     phm_array->add(phm_entry);
        // }
        // Poco::JSON::Object::Ptr phm = new Poco::JSON::Object;
        // phm->set("data", phm_array);
        // response->set("phm", phm);
        //
        // // wtf_health_data
        // Poco::JSON::Array::Ptr health_array = new Poco::JSON::Array;
        // std::vector<std::pair<std::string, std::string>> health_data = {
        //     {"Drive_motor", "20000"},
        //     {"Caster_wheel", "10000"},
        //     {"Bearing_32006", "6162846"},
        //     {"Bearing_32003", "277897"},
        //     {"Charge_relay", "200000"},
        //     {"Drive_motor_relay", "200000"},
        //     {"TBM_LIFT_Life_ServoMotor", "30000"},
        //     {"TBM_LIFT_Life_Bearing_6000ZZC3", "1125075"},
        //     {"TBM_LIFT_Life_BollScrew", "9320000"},
        //     {"TBM_LIFT_Fill_BollScrew", "4320"},
        //     {"TBM_LIFT_Life_LM_Guide", "84700000"},
        //     {"TBM_LIFT_Fill_LM_Guide", "4320"},
        //     {"TBM_PLCStopper_Life_StopperRelay", "2000000"},
        //     {"TBM_STACK_Life_Bearing_6800ZZ", "3145321"},
        //     {"TBM_STACK_Life_Actuator", "10000"},
        //     {"TBM_SKID_Life_Actuator", "40000"},
        //     {"TBM_CONV_Life_Shover", "300000"},
        //     {"TBM_CONV_Life_MotorRoller", "17640"}};
        // for (auto& [name, data] : health_data) {
        //     Poco::JSON::Object::Ptr entry = new Poco::JSON::Object;
        //     entry->set("name", name);
        //     entry->set("type", "long");
        //     entry->set("data", data);
        //     health_array->add(entry);
        // }
        //
        // Poco::JSON::Object::Ptr wtf_health = new Poco::JSON::Object;
        // wtf_health->set("data", health_array);
        // response->set("wtf_health_data", wtf_health);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Exception in amr_info: " << ex.displayText();
    }
}
