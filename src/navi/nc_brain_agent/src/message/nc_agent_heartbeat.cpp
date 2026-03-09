#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/message/nc_agent_heartbeat.h>

using Poco::JSON::Object;
using namespace NaviFra;

NcAgentHearbeat::NcAgentHearbeat()
{
}
NcAgentHearbeat::NcAgentHearbeat(std::string id, Poco::Timestamp time, bool is_slam)
    : id_(id)
    , timestamp_(time)
    , is_slam_(is_slam)
{
}
NcAgentHearbeat::~NcAgentHearbeat()
{
}
void NcAgentHearbeat::setID(std::string id)
{
    id_ = id;
}
void NcAgentHearbeat::setTimestamp(Poco::Timestamp time)
{
    timestamp_ = time;
}
void NcAgentHearbeat::setIsSlam(bool is)
{
    is_slam_ = is;
}
void NcAgentHearbeat::setVersion(const std::string& version)
{
    version_ = version;
}

std::string NcAgentHearbeat::toString()
{
    Object data;

    data.set("id", id_);
    data.set("timestamp", timestamp_.raw());
    data.set("is_slam", is_slam_);
    data.set("version", version_);
    
    std::ostringstream ostr;
    data.stringify(ostr);

    return ostr.str();
}