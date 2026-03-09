#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/manager/publish_manager.h>
#include <nc_wia_agent/action/nc_action_amr_basic.h>
#include <nc_wia_agent/data/status_channel.h>

using namespace NaviFra;

NcActionAMRBasic::NcActionAMRBasic()
{
}

NcActionAMRBasic::~NcActionAMRBasic()
{
}

std::string NcActionAMRBasic::implName()
{
    return "NcActionAMRBasic";
}

void NcActionAMRBasic::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        //    std::string datetime = obj->getValue<std::string>("datetime");
        int interval = obj->getValue<int>("interval");

        // 채널 주기 설정 및 활성화
        PublisherManager::instance().setChannelInterval((int)PUBLISH_CHANNEL::CHANNEL_BASIC, interval);

        if (0 <= interval)
            PublisherManager::instance().activate((int)PUBLISH_CHANNEL::CHANNEL_BASIC);
        else
            PublisherManager::instance().deactivate((int)PUBLISH_CHANNEL::CHANNEL_BASIC);
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Exception: " << ex.displayText();
    }
}
