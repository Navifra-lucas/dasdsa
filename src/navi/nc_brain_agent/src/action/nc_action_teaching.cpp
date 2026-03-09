#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/DynamicStruct.h>
#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_teaching.h>
#include <nc_brain_agent/data/nc_brain_map.h>

using namespace NaviFra;

NcActionTeaching::NcActionTeaching()
{
}

NcActionTeaching::~NcActionTeaching()
{
}

std::string NcActionTeaching::implName()
{
    return "NcActionTeaching";
}

void NcActionTeaching::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");

    std::string strdata = obj->has("data") ? obj->get("data").convert<std::string>() : "";
    if (strdata.compare("") == 0) {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
    else {
        std::string result = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->teachingUpdate(obj->getObject("data"));
        Object::Ptr data_result = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getTeachedData();
        sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), std::move(data_result), result);

        updateBrainMap("nodelink");
    }
}
