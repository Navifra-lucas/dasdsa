#include "nc_brain_agent/initializer/map_sync_initializer.h"

#include "core_agent/core/navicore.h"
#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_info.h"
#include "core_agent/data/robot_status.h"
#include "core_agent/util/config.h"
#include "nc_brain_agent/data/nc_brain_map.h"
#include "nc_brain_agent/net/nc_rest_api_utils.h"
#include "util/logger.hpp"


namespace NaviFra {

void MapSyncInitializer::initialize()
{
    NLOG(info) << "MapSyncInitializer - Start";
    auto token = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken();
    s_path_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    // 디렉토리 보장

    // area 사용모드 체크
    bool b_use_area_mode = true;
    auto res_areas = Net::HTTP::get("/scan-maps/areas", token);
    if (std::get<1>(res_areas) != Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        LOG_ERROR("Failed to get areas: %s", std::get<2>(res_areas).c_str());
        b_use_area_mode = false;
    }

    if( b_use_area_mode )  // backend에서 area 사용모드가 활성화된 경우
    {
        LOG_INFO("Area mode is enabled. Skipping map synchronization.");
        InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->updateAreaPortal();
        
    }
    else // 기존 레거시 모드
    {
        LOG_ERROR("Area mode is disabled. Proceeding with map synchronization.");
        // ----------- Main Map Info 요청 -----------
        auto res_map = Net::HTTP::get("/scan-maps/main-info", token);
        std::string body = std::get<0>(res_map);
        auto status = std::get<1>(res_map);
        std::string reason = std::get<2>(res_map);

        if (status != Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
            LOG_ERROR("backend /scan-maps/main-info status error : Status %d Reason %s", (int)status, reason.c_str());
            return;
        }

        LOG_TRACE("Agent - BackEnd Get -> /scan-maps/main-info - Passed");

        auto mapData = JSONParse(body);
        auto brainMap = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY);
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

        // 서버 revision 설정
        brainMap->setMainRevision(mapData->getObject("item")->get("revision").convert<std::string>());

        // ----------- 로컬 맵 초기화 체크 -----------
        if (!brainMap->initialize()) {
            LOG_INFO("Local map not initialized. Downloading main map...");

            res_map = Net::HTTP::get("/scan-maps/main", token);
            body = std::get<0>(res_map);
            status = std::get<1>(res_map);
            reason = std::get<2>(res_map);

            if (status != Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                LOG_ERROR("backend /scan-maps/main status error : Status %d Reason %s", (int)status, reason.c_str());
                return;
            }

            mapData = JSONParse(body);
            brainMap->localMapUpdate(mapData->getObject("item"));
            brainMap->teachingJsonInit(brainMap->getMainRevision());

    #ifndef CPP_UNIT_TEST
            updateBrainMap();
    #endif
        }
        else {
            // ----------- 리비전 비교 및 업데이트 -----------
            if (brainMap->getLocalRevision() != brainMap->getMainRevision() &&
                InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->isJobActive()) {
                LOG_INFO("Map revision mismatch. Downloading updated main map...");

                res_map = Net::HTTP::get("/scan-maps/main", token);
                body = std::get<0>(res_map);
                status = std::get<1>(res_map);
                reason = std::get<2>(res_map);

                if (status != Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                    LOG_ERROR("backend /scan-maps/main status error : Status %d Reason %s", (int)status, reason.c_str());
                    return;
                }

                mapData = JSONParse(body);
                brainMap->localMapUpdate(mapData->getObject("item"));
                brainMap->teachingJsonInit(brainMap->getMainRevision());

    #ifndef CPP_UNIT_TEST
                updateBrainMap();
    #endif
            }
        }

        // ----------- RobotInfo 세팅 업데이트 -----------
        robotInfo->setSetting(std::atoi(brainMap->getLocalRevision().c_str()), brainMap->getKeyNodesSize(), brainMap->getTeachedNodesSize());

        LOG_INFO("Map synchronization completed. Revision: %s", brainMap->getMainRevision().c_str());
    }
}

}  // namespace NaviFra
