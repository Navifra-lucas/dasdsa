#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_fork_wide_cmd.h>

using namespace NaviFra;

NcActionForkWideCmd::NcActionForkWideCmd()
{
    pub_fork_ = nh_.advertise<core_msgs::WiaForkInfo>("/forkinfo", 10, false);
}

NcActionForkWideCmd::~NcActionForkWideCmd()
{
}

ForkWideStatus NcActionForkWideCmd::checkForkWideStatus(int height)
{
    try {
        if (height < 0) {
            return ForkWideStatus::VALUE_ERROR;
        }
        return ForkWideStatus::IDLE;
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Poco tilt Exception: " << ex.displayText();
        return ForkWideStatus::UNKNOWN_ERROR;
    }
}

std::string NcActionForkWideCmd::implName()
{
    return "NcActionForkWideCmd";
}

void NcActionForkWideCmd::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Fork wide command received from RID: %s", rid);

        int height = obj->getValue<int>("height");

        ForkWideStatus status = checkForkWideStatus(height);

        int err_code = 0;
        bool success;

        if (status == ForkWideStatus::IDLE) {
            err_code = 0;
            success = true;
            core_msgs::WiaForkInfo fork_info;
            fork_info.n_fork_height = -1;
            fork_info.n_fork_wide = height;
            fork_info.f_fork_tilt = -91.0f;
            pub_fork_.publish(fork_info);
        }
        else if (status == ForkWideStatus::VALUE_ERROR) {
            err_code = 2;
            success = false;
        }
        else {
            err_code = 4;
            success = false;
        }

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "fork_wide_cmd");
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
        NLOG(error) << "Fork Wide Exception: " << ex.displayText();
    }
}
