#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/nc_action_shutdown.h>

using namespace NaviFra;

NcActionShutdown::NcActionShutdown()
{
}

NcActionShutdown::~NcActionShutdown()
{
}

std::string NcActionShutdown::implName()
{
    return "NcActionShutdown";
}

void NcActionShutdown::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    NLOG(info) << "Recv system shutdown request.";
    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
    LOG_INFO("system shutdown");
#ifdef __unix__  // linux 일때 만
    sync();
    int result = std::system("sudo shutdown now");
#endif
}
