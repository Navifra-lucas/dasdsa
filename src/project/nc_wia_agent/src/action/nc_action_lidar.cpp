#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/manager/publish_manager.h>
#include <nc_wia_agent/action/nc_action_lidar.h>
#include <nc_wia_agent/data/status_channel.h>

using namespace NaviFra;

NcActionLidar::NcActionLidar()
{
}

NcActionLidar::~NcActionLidar()
{
}

std::string NcActionLidar::implName()
{
    return "NcActionLidar";
}

void NcActionLidar::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->optValue<std::string>("msg_time", "");
        int msg_id = obj->getValue<int>("msg_id");
        int lidar_range = obj->optValue<int>("lidar_range", 0);

        bool success = true;  // 예제에서는 성공했다고 가정
        int err_code = 0;  // 에러 없음을 뜻함

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("err_code", err_code);
        response->set("success", success);
        response->set("Cmd", "lidar");
        std::vector<float> lidarData = status->getLidarData();
        Poco::JSON::Array::Ptr rangesArray = new Poco::JSON::Array;
        NLOG(info) << "lidardata size is ->> " << lidarData.size();
        for (float r : lidarData) {
            if (r <= lidar_range && r > 0) {  // 특정 거리 이내만 추가
                rangesArray->add(r);
            }
        }
        response->set("ranges", rangesArray);

        if (msg_time.empty()) {
            msg_time = Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i");
        }
        response->set("msg_time", msg_time);
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "lidar Exception: " << ex.displayText();
    }
}
