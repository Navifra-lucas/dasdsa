
#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_info.h>
#include <nc_brain_agent/action/async/nc_async_action_update_map.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>

using namespace NaviFra;

using Poco::DynamicStruct;
using Poco::JSON::Object;
using Poco::Net::HTTPResponse;

NcAsyncActionMapUpdate::NcAsyncActionMapUpdate(std::string robotid, std::string uuid, std::string token, Poco::JSON::Object::Ptr obj)
    : robotid_(robotid)
    , uuid_(uuid)
    , token_(token)
    , obj_(obj)
{
}

NcAsyncActionMapUpdate::~NcAsyncActionMapUpdate()
{
}

void NcAsyncActionMapUpdate::run()
{
    try {
        if (uuid_.empty() || token_.empty()) {
            LOG_ERROR("NcAsyncActionMapUpdate empty uuid or token : uuidd [ %s ], token [ %s ]", uuid_.c_str(), token_.c_str());
            delete this;
            return;
        }

        if (obj_.get() == nullptr) {
            LOG_ERROR("NcAsyncActionMapUpdate JSON data is NULL");
            delete this;
            return;
        }

        DynamicStruct data = *obj_;
        LOG_TRACE(
            "updatemap revision %d, local revision %s", (int)data["data"]["revision"],
            InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getLocalRevision().c_str());
        int revision = (int)data["data"]["revision"];
        // revision 같은데 세이브 하면 날라간다.

        std::string uri = "/scan-maps/" + data["data"]["id"];
        std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res = Net::HTTP::get(uri, token_);
        std::string is = std::get<0>(res);
        HTTPResponse::HTTPStatus status = std::get<1>(res);

        if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
            Poco::JSON::Object::Ptr map = JSONParse(is);
            InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->localMapUpdate(map->getObject("item"));
            InMemoryRepository::instance()
                .get<NcBrainMap>(NcBrainMap::KEY)
                ->teachingJsonInit(InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getMainRevision());

            // NcRobotAgent::get().onUpdatedMap(); 이거 어떻게 걷어 내지?
            updateBrainMap();

            InMemoryRepository::instance()
                .get<RobotInfo>(RobotInfo::KEY)
                ->setSetting(
                    std::atoi(InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getLocalRevision().c_str()),
                    InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getKeyNodesSize(),
                    InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getTeachedNodesSize());

            Object response, emptyData;
            response.set("uuid", uuid_);
            response.set("id", robotid_);
            response.set("result", "success");
            response.set("data", emptyData);

            std::ostringstream oss;
            response.stringify(oss);
            MessageBroker::instance().publish("robot.response:" + robotid_, oss.str());
        }
        else {
            LOG_ERROR("NcAsyncActionMapUpdate get api path /scan-maps/%s error", data["data"]["id"].extract<std::string>().c_str());
            Object response, emptyData;
            response.set("uuid", uuid_);
            response.set("id", robotid_);
            response.set("result", "fail");
            response.set("message", "Maps cannot be downloaded. MapID: " + data["data"]["id"]);
            response.set("data", emptyData);

            std::ostringstream oss;
            response.stringify(oss);
            MessageBroker::instance().publish("robot.response:" + robotid_, oss.str());
        }
    }
    catch (std::exception ex) {
        LOG_ERROR("NcAsyncActionMapUpdate error %s", ex.what());
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
