#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_conv_cmd.h>

using namespace NaviFra;

NcActionConvCMD::NcActionConvCMD()
{
}

NcActionConvCMD::~NcActionConvCMD()
{
}

std::string NcActionConvCMD::implName()
{
    return "NcActionConvCMD";
}

void NcActionConvCMD::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");
        int mode = obj->getValue<int>("mode");

        NLOG(info) << Poco::format("Received conveyor command: mode=%d", mode);

        // 실제 컨베이어 제어 로직 호출 (예시)
        // bool success = conveyorController->run(mode);
        bool success = true;  // 예시로 성공 처리
        int err_code = 0;

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("err_code", err_code);
        response->set("success", success);
        response->set("Cmd", "conv_cmd");
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
        NLOG(error) << "ConvCmd Exception: " << ex.displayText();
    }
}
