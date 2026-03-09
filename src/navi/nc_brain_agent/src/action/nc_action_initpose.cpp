#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_initpose.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include "nc_brain_agent/data/nc_brain_map.h"

using namespace NaviFra;

NcActionInitPose::NcActionInitPose()
{
}

NcActionInitPose::~NcActionInitPose()
{
}

std::string NcActionInitPose::implName()
{
    return "NcActionInitPose";
}

void NcActionInitPose::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");

    Position position;
    Orientation orientation;

    // area_id 속성이 있을 때만 실행
    if (data->has("area_id")) {
        std::string area_id = data->get("area_id").convert<std::string>();
        if(!area_id.empty()) 
        {
            // area_id가 있는 경우, 해당 area의 초기 위치로 설정
            changeArea(area_id);
            // InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->ChageCurrentArea(area_id);
            LOG_INFO("Changed area to %s", area_id.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    Poco::JSON::Object::Ptr positionData = data->getObject("position");
    position.x = (float)positionData->get("x");
    position.y = (float)positionData->get("y");
    position.z = (float)positionData->get("z");

    Poco::JSON::Object::Ptr orientationData = data->getObject("orientation");
    orientation.x = (float)orientationData->get("x");
    orientation.y = (float)orientationData->get("y");
    orientation.z = (float)orientationData->get("z");
    orientation.w = (float)orientationData->get("w");
    bool correct_position = data->get("correct_position");

    initialPose(position, orientation, correct_position);

    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}
