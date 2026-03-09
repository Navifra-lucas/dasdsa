#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_fork_tilt_cmd.h>

using namespace NaviFra;

NcActionForkTiltCmd::NcActionForkTiltCmd()
{
    pub_fork_ = nh_.advertise<core_msgs::WiaForkInfo>("/forkinfo", 10, false);
}

NcActionForkTiltCmd::~NcActionForkTiltCmd()
{
}

TiltStatus NcActionForkTiltCmd::checkTiltStatus(float angle)
{
    try {
        if (angle < 0)  // 정해진 범위 값 수정해야함
        {
            return TiltStatus::VALUE_ERROR;
        }

        return TiltStatus::IDLE;
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Poco tilt Exception: " << ex.displayText();
        return TiltStatus::UNKNOWN_ERROR;
    }
}

std::string NcActionForkTiltCmd::implName()
{
    return "NcActionForkTiltCmd";
}

void NcActionForkTiltCmd::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Fork Tilt command received from RID: %s", rid);

        float angle = obj->getValue<float>("angle");

        int err_code = 0;
        bool success;

        TiltStatus status = checkTiltStatus(angle);

        if (status == TiltStatus::IDLE) {
            err_code = 0;
            success = true;
            core_msgs::WiaForkInfo fork_info;
            fork_info.n_fork_height = -1;
            fork_info.n_fork_wide = -1;
            fork_info.f_fork_tilt = angle;
            pub_fork_.publish(fork_info);
        }
        else if (status == TiltStatus::VALUE_ERROR) {
            err_code = 2;
            success = false;
        }
        else {
            err_code = 4;
            success = false;
        }

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "fork_tilt_cmd");
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
        NLOG(error) << "Fork tilt Exception: " << ex.displayText();
    }
}
