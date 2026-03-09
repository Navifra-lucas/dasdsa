#include "ros1/docking_path_generator/docking_info.hpp"

#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

#include "logger/logger.h"

namespace NVFR {

bool ParseJsonPose(
    const std::string& s_key,
    const Poco::JSON::Object::Ptr jsonObject,
    Pose* o_pose_ptr)
{
    // exceptions
    if (!jsonObject) {
        LOG_ERROR("jsonObject is nullptr (key: {})",
            s_key.c_str());
        return false;
    }
    if (!jsonObject->has(s_key)) {
        LOG_ERROR("There is not key: {}",
            s_key.c_str());
        return false;
    }
    auto obj = jsonObject->getObject(s_key);
    if (!obj->has("x") || !obj->has("y") || !obj->has("deg")) {
        LOG_ERROR("There is not key (x | y | deg) in key: {}",
            s_key.c_str());
        return false;
    }

    try {
        o_pose_ptr->SetXm(obj->get("x").convert<float>());
        o_pose_ptr->SetYm(obj->get("y").convert<float>());
        o_pose_ptr->SetDeg(obj->get("deg").convert<float>());
    }
    catch (Poco::Exception &ex) {
        LOG_ERROR("Error: %s", ex.displayText().c_str());
        return false;
    }

    return true;
}

bool DockingInfo::ParseJson(const std::string& s_data)
{
    std::string json_data = R"({
        "n_mission_type": 0,
        "o_fNode": {
            "x": 0.0,
            "y": 0.0,
            "deg": 0.0
        },
        "o_object2goal": {
            "x": 0.0,
            "y": 0.0,
            "deg": 0.0
        },
        "o_pose": {
            "x": 0.0,
            "y": 0.0,
            "deg": 0.0
        },
        "b_local": false
    })";

    // parse JSON
    try {
        Poco::JSON::Object::Ptr jsonObject = nullptr;
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(s_data.c_str());
        jsonObject = result.extract<Poco::JSON::Object::Ptr>();
        if (jsonObject->has("n_mission_type")) {
            n_mission_type = static_cast<DockingInfo::CMD>(
                jsonObject->get("n_mission_type").convert<int>());
        }
        else {
            LOG_ERROR("There is not key: n_mission_type");
            return false;
        }
        if (!ParseJsonPose("o_fNode", jsonObject, &o_fNode)) {
            LOG_ERROR("Fail to parse pose (key: o_fNode)");
            return false;
        }
        if (!ParseJsonPose("o_object2goal", jsonObject, &o_object2goal)) {
            LOG_ERROR("Fail to parse pose (key: o_object2goal)");
            return false;
        }
        if (!ParseJsonPose("o_pose", jsonObject, &o_pose)) {
            LOG_ERROR("Fail to parse pose (key: o_pose)");
            return false;
        }
        if (jsonObject->has("b_local")) {
            b_local = jsonObject->get("b_local").convert<bool>();
        }
        else {
            LOG_ERROR("There is not key: b_local");
            return false;
        }
    }
    catch (Poco::Exception &ex) {
        LOG_ERROR("Error: %s", ex.displayText().c_str());
        return false;
    }

    return true;
}

} // namespace NVFR
