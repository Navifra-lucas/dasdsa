#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/DynamicStruct.h>
#include <nc_brain_agent/action/nc_action_teached.h>
#include <nc_brain_agent/data/nc_brain_map.h>

using namespace NaviFra;

NcActionTeached::NcActionTeached()
{
}

NcActionTeached::~NcActionTeached()
{
}

std::string NcActionTeached::implName()
{
    return "NcActionTeached";
}

void NcActionTeached::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    std::string strdata = obj->has("data") ? obj->get("data").convert<std::string>() : "";
    if (strdata.compare("") == 0) {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
    else {
        if (obj->getObject("data")->size() == 0) {
            Object::Ptr data_result = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getTeachedData();
            sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), std::move(data_result));
            return;
        }
        std::string result = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->teachingUpdate(obj->getObject("data"));
        Object::Ptr data_result = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getTeachedData();

        sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), std::move(data_result));
    }
}
