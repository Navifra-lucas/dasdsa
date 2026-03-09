
#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <nc_brain_agent/action/async/nc_async_action_sync_map.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>
#include "core_agent/data/robot_status.h"

using namespace NaviFra;

using Poco::DynamicStruct;
using Poco::JSON::Object;
using Poco::Net::HTTPResponse;

NcAsyncActionSyncMap::NcAsyncActionSyncMap(std::string robotid, std::string uuid, std::string token, Poco::JSON::Object::Ptr obj)
    : robotid_(robotid)
    , uuid_(uuid)
    , token_(token)
    , obj_(obj)
{
}

NcAsyncActionSyncMap::~NcAsyncActionSyncMap()
{
}

void NcAsyncActionSyncMap::run()
{
    try {
        if (uuid_.empty() || token_.empty()) {
            LOG_ERROR("NcAsyncActionSyncMap empty uuid or token : uuidd [ %s ], token [ %s ]", uuid_.c_str(), token_.c_str());
            delete this;
            return;
        }

        if (obj_.get() == nullptr) {
            LOG_ERROR("NcAsyncActionSyncMap JSON data is NULL");
            delete this;
            return;
        }

        NLOG(info) << "MapSyncInitializer - Start";
        auto token = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken();
        // 디렉토리 보장
    
        // area 사용모드 체크
        auto res_areas = Net::HTTP::get("/scan-maps/areas", token);
    
        LOG_INFO("Area synchronization.");
        InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->updateAreaPortal();
        Object response, emptyData;
        response.set("uuid", uuid_);
        response.set("id", robotid_);
        response.set("result", "success");
        response.set("data", emptyData);

        std::ostringstream oss;
        response.stringify(oss);
        MessageBroker::instance().publish("robot.response:" + robotid_, oss.str());
    }
    catch (std::exception ex) {
        LOG_ERROR("NcAsyncActionSyncMap error %s", ex.what());
        Object response, emptyData;
        response.set("uuid", uuid_);
        response.set("id", robotid_);
        response.set("result", "fail");
        response.set("message", "throw exception Map update" + std::string(ex.what()));
        response.set("data", emptyData);

        std::ostringstream oss;
        response.stringify(oss);
        MessageBroker::instance().publish("robot.response:" + robotid_, oss.str());
    }

    delete this;
}
