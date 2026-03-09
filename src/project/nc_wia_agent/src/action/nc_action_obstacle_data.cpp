#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/manager/publish_manager.h>
#include <nc_wia_agent/action/nc_action_obstacle_data.h>
#include <nc_wia_agent/data/status_channel.h>

using namespace NaviFra;

NcActionObstacleData::NcActionObstacleData()
{
}

NcActionObstacleData::~NcActionObstacleData()
{
}

std::string NcActionObstacleData::implName()
{
    return "NcActionObstacleData";
}

void NcActionObstacleData::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        bool success = true;  // 예제에서는 성공했다고 가정
        int err_code = 0;  // 에러 없음을 뜻함

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("err_code", err_code);
        response->set("success", success);
        response->set("Cmd", "obstacle_data");
        std::vector<float> obstacleData = status->getObstacleData();
        Poco::JSON::Array::Ptr rangesArray = new Poco::JSON::Array;
        for (float r : obstacleData) {
            rangesArray->add(r);
        }
        response->set("ranges", rangesArray);
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "LiftCmd Exception: " << ex.displayText();
    }
}
