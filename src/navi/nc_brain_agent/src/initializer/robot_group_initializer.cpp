#include "nc_brain_agent/initializer/robot_group_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_info.h"
#include "nc_brain_agent/net/nc_rest_api_utils.h"
#include "nc_brain_agent/utils/nc_agent_config.h"
#include "util/logger.hpp"

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/Net/HTTPResponse.h>

namespace NaviFra {

void RobotGroupInitializer::initialize()
{
#ifndef CPP_UNIT_TEST
    std::string drivingType = NcAgentConfig::get().getDrivingType_cd();

    if (drivingType.size() < 3) {
        LOG_ERROR("Invalid driving type code: %s", drivingType.c_str());
        return;
    }

    std::string subDrivingType = drivingType.substr(0, drivingType.size() - 2);

    // ----------- 백엔드 호출 -----------
    auto res = Net::HTTP::get("/group-codes/" + subDrivingType);
    std::string body = std::get<0>(res);
    auto status = std::get<1>(res);
    std::string reason = std::get<2>(res);

    if (status != Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        LOG_ERROR(
            "backend /group-codes/%s status error : Status %d Reason %s", subDrivingType.c_str(), static_cast<int>(status), reason.c_str());
        return;
    }

    // ----------- JSON 처리 -----------
    auto data = JSONParse(body);
    auto list = data->getArray("list");
    if (!list) {
        LOG_ERROR("Invalid response: missing list field");
        return;
    }

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    for (size_t i = 0; i < list->size(); i++) {
        Poco::JSON::Object::Ptr obj = list->getObject(i);
        std::string codeId = obj->get("code_id").convert<std::string>();
        std::string groupCodeId = obj->get("group_code_id").convert<std::string>();

        if (codeId == drivingType && groupCodeId == subDrivingType) {
            robotInfo->setRobotType(obj->get("code_kname").convert<std::string>());
            LOG_INFO("Robot type set to %s", robotInfo->getRobotType().c_str());
            break;
        }
    }

    LOG_TRACE("Agent - BackEnd Get -> /group-codes/ - Passed");
#endif
}

}  // namespace NaviFra
