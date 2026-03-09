#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_lift_cmd.h>
#include <task_msgs/Loading.h>

using namespace NaviFra;

NcActionLiftCMD::NcActionLiftCMD()
{
    send_loading_ = nh.advertise<task_msgs::Loading>("/task_manager/lift_control", 10, false);
}

NcActionLiftCMD::~NcActionLiftCMD()
{
}

std::string NcActionLiftCMD::implName()
{
    return "NcActionLiftCMD";
}

void NcActionLiftCMD::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");
        int mode = obj->getValue<int>("mode");
        NLOG(info) << "lift mode is : " << mode;
        task_msgs::Loading msg;

        if (mode == 1) {
            msg.type = "loading";
            msg.level = 0;
        }
        else if (mode == -1) {
            msg.type = "unloading";
            msg.level = 0;
        }
        send_loading_.publish(msg);
        NLOG(info) << "publish!";
        // int height = obj->getValue<int>("height");

        // NLOG(info) << Poco::format("Received lift command: mode=%d, height=%d", mode, height);

        // 실제 리프트 동작 함수 호출 (예시)
        // bool success = liftController->executeLiftCommand(mode, height);
        bool success = true;  // 예제에서는 성공했다고 가정
        int err_code = 0;  // 에러 없음을 뜻함

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("err_code", err_code);
        response->set("success", success);
        response->set("Cmd", "lift_cmd");
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
