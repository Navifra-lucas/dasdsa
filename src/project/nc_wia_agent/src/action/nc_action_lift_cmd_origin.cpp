#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_lift_cmd_origin.h>

using namespace NaviFra;

NcActionLiftOrigin::NcActionLiftOrigin()
{
}

NcActionLiftOrigin::~NcActionLiftOrigin()
{
}

std::string NcActionLiftOrigin::implName()
{
    return "NcActionLiftOrigin";
}

void NcActionLiftOrigin::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << "Received lift origin command";

        // 실제 리프트 동작 함수 호출 (예시)
        // bool success = liftController->executeLiftCommand(mode, height);
        bool success = true;  // 예제에서는 성공했다고 가정
        int err_code = 0;  // 에러 없음을 뜻함

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("err_code", err_code);
        response->set("success", success);
        response->set("Cmd", "lift_cmd_origin");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "LiftCmd Exception: " << ex.displayText();
    }
}
