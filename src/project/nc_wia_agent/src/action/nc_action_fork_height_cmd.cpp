#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_fork_height_cmd.h>

using namespace NaviFra;

NcActionForkHeightCmd::NcActionForkHeightCmd()
{
    pub_fork_ = nh_.advertise<core_msgs::WiaForkInfo>("/forkinfo", 10, false);
}

NcActionForkHeightCmd::~NcActionForkHeightCmd()
{
}

std::string NcActionForkHeightCmd::implName()
{
    return "NcActionForkHeightCmd";
}

ForkStatus NcActionForkHeightCmd::checkForkStatus(int mode, int height)
{
    // int n_now_height; 리프트 스테이터스 긁어오기

    try {
        if (height < 0)
            return ForkStatus::VALUE_ERROR;
        // else if (mode == 1 && n_now_height > height) {
        // return ForkStatus::MODE_ERROR;
        // }
        // else if (mode == -1 && n_now_height < height) {
        // return ForkStatus::MODE_ERROR;
        // }
        return ForkStatus::IDLE;
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Poco tilt Exception: " << ex.displayText();
        return ForkStatus::UNKNOWN_ERROR;
    }
}

void NcActionForkHeightCmd::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Fork Height command received from RID: %s", rid);

        int mode = obj->getValue<int>("mode");
        int height = obj->getValue<int>("height");

        ForkStatus status = checkForkStatus(mode, height);

        int err_code = 0;
        bool success;

        if (status == ForkStatus::IDLE) {
            err_code = 0;
            success = true;
            //토픽으로 제어?????
            core_msgs::WiaForkInfo fork_info;
            fork_info.n_fork_height = height;
            fork_info.n_fork_wide = -1;
            fork_info.f_fork_tilt = -91.0f;
            pub_fork_.publish(fork_info);
        }
        else if (status == ForkStatus::MODE_ERROR) {
            err_code = 1;
            success = false;
        }
        else if (status == ForkStatus::VALUE_ERROR) {
            err_code = 2;
            success = false;
        }
        else {
            err_code = 4;
            success = false;
        }

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "fork_height_cmd");
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
        NLOG(error) << "Fork Height Exception: " << ex.displayText();
    }
}
