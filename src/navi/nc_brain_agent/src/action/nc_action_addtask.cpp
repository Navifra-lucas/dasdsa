#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/DynamicStruct.h>
#include <core_agent/core/navicore.h>
#include <core_msgs/AddTask.h>
#include <nc_brain_agent/action/nc_action_addtask.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include <nc_brain_agent/nc_robot_agent.h>

using namespace NaviFra;
using Poco::DynamicStruct;

NcActionAddTask::NcActionAddTask()
{
}

NcActionAddTask::~NcActionAddTask()
{
}

std::string NcActionAddTask::implName()
{
    return "NcActionAddTask";
}

void NcActionAddTask::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    using namespace Poco::JSON;
    using Poco::Dynamic::Var;

    Array::Ptr inputData = obj->getArray("data");

    // 최종적으로 보낼 JSON Object
    Object::Ptr outputObj = new Object;
    outputObj->set("action", "add_task");

    // 새로운 uuid 생성
    std::string obj_uuid = obj->getValue<std::string>("uuid");
    outputObj->set("uuid", obj_uuid);

    // inputData 전체를 바로 복사
    outputObj->set("data", inputData);

    // move 타입만 path_nodes 추가
    auto dataArray = outputObj->getArray("data");
    for (size_t i = 0; i < dataArray->size(); ++i) {
        Object::Ptr task = dataArray->getObject(i);
        std::string type = task->getValue<std::string>("type");
        std::string uuid = task->getValue<std::string>("uuid");
        NLOG(info) << "add_task UUID: " << uuid;

        if (type == "move" || type == "turn" || type == "action") {
            Array::Ptr vecMoveData = new Array;
            vecMoveData->add(task->getValue<std::string>("start_node"));
            if (task->getValue<std::string>("start_node") != task->getValue<std::string>("end_node"))
                vecMoveData->add(task->getValue<std::string>("end_node"));
            task->set("path_nodes", vecMoveData);
            task->set("finish_angle", task->getValue<float>("finish_angle"));
        }
    }

    // 최종 JSON String으로 변환
    std::stringstream oss;
    outputObj->stringify(oss);

    NLOG(info) << "add_task: " << oss.str();
    ProcessResult result = requestStringSrv(NcBrainCommand::BRAIN_TASK_ADD, oss.str());
    if (result.success) {
        sendResponseSuccess(source, obj_uuid);
    }
    else {
        sendResponseSuccess(source, obj_uuid, "fail", result.message);
    }
}