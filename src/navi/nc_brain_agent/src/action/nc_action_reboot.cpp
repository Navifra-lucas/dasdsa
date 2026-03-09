#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/nc_action_reboot.h>

using namespace NaviFra;

NcActionReboot::NcActionReboot()
{
}

NcActionReboot::~NcActionReboot()
{
}

std::string NcActionReboot::implName()
{
    return "NcActionReboot";
}

void NcActionReboot::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
    LOG_INFO("system reboot");
#ifdef __unix__  // linux 일때 만
    sync();
    int result = std::system("sudo reboot now");
#endif
}
