#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/async/nc_async_action_slam_start.h>
#include <nc_brain_agent/action/nc_action_start_slam.h>

using namespace NaviFra;

NcActionStartSLAM::NcActionStartSLAM()
{
}

NcActionStartSLAM::~NcActionStartSLAM()
{
}

std::string NcActionStartSLAM::implName()
{
    return "NcActionStartSLAM";
}

void NcActionStartSLAM::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    // if (robotInfo->getStatus() != "idle") {
    //     LOG_ERROR("robot status is not idle. can not action on %s", action.c_str());
    //     sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail");
    //     // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
    //     status."); return;
    // }

    auto r = new NcAsyncActionSLAMStart(obj->get("uuid").convert<std::string>());
    Poco::ThreadPool::defaultPool().start(*r);
}
