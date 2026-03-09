#include "nc_brain_agent/initializer/robot_verify_initializer.h"

#include "nc_brain_agent/nc_brain_agent.h"
#include "nc_brain_agent/net/nc_rest_api_utils.h"
#include "nc_brain_agent/utils/nc_agent_config.h"

#include <Poco/JSON/Object.h>
#include <Poco/Net/HTTPResponse.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/redis/redis_reader.h>
#include <core_agent/util/config.h>

using Poco::Net::HTTPResponse;

namespace NaviFra {

void RobotVerifyInitializer::initialize()
{
    std::string backend_port = Config::instance().getString("backend_port", "5000");
    std::string redis_host = Config::instance().getString("redis_host", "127.0.0.1");
    std::string redis_port = Config::instance().getString("redis_port", "5000");
    std::string redis_pass = Config::instance().getString("redis_passwd", "navifra1@3$");

    // Redis에서 등록 코드 읽기
    bool use_auth = Config::instance().getBool("use_auth", true);
    bool use_redis = Config::instance().getBool("use_redis", false);

    NaviFra::RedisReader::Ptr redisReader;

    redisReader.reset(new NaviFra::RedisReader(use_auth));
    if (redisReader->initialize(redis_host.c_str(), std::atoi(redis_port.c_str()), redis_pass.c_str())) {
        LOG_INFO("RedisReader connected");
    }
    else {
        LOG_ERROR("can't initialize RedisReader connected");
        return;
    }
    std::string robot_acs_reg_code = redisReader ? redisReader->GET("robot_acs_reg_code") : "";

    // 등록 데이터 생성
    Poco::JSON::Object robot_create_data;
    std::string modelType = Config::instance().getString("model_type", "D10001");
    std::string drivingType = Config::instance().getString("drive_type", "D30001");
    std::string ipAddress = Config::instance().getString("robot_ip", "127.0.0.1");

    robot_create_data.set("model_type_cd", modelType);
    robot_create_data.set("driving_type_cd", drivingType);
    robot_create_data.set("ip_addr", ipAddress);
    robot_create_data.set("robot_acs_reg_code", robot_acs_reg_code);

    LOG_TRACE("Agent - robot_create_data - Passed");

    // 백엔드 호출
    auto res = Net::HTTP::post("/robots/create", robot_create_data);
    std::string body = std::get<0>(res);
    auto status = std::get<1>(res);
    std::string reason = std::get<2>(res);

    auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

    if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_CREATED || status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        Poco::JSON::Object::Ptr data = JSONParse(body);

        auto item = data->getObject("item");
        std::string id = item->get("id").convert<std::string>();
        std::string token = item->get("token").convert<std::string>();

        robotStatus->setID(id);
        robotStatus->setToken(token);
        robotInfo->setID(id);

        NcAgentConfig::get().initializeRobot();
        NcAgentConfig::get().setRobotID(id);
        NcAgentConfig::get().setToken(token);
    }
    else if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_CONFLICT || status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_BAD_REQUEST) {
        // 이미 등록된 로봇 처리
        if (!NcAgentConfig::get().getToken().empty()) {
            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
            auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
            robotStatus->setID(NcAgentConfig::get().getRobotID());
            robotStatus->setToken(NcAgentConfig::get().getToken());
            robotInfo->setID(NcAgentConfig::get().getRobotID());

            std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res = Net::HTTP::get(
                "/robots/" + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());

            std::string is = std::get<0>(res);
            HTTPResponse::HTTPStatus status = std::get<1>(res);
            std::string reason = std::get<2>(res);

            if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                Poco::JSON::Object::Ptr dataRobot = JSONParse(is);
                robotStatus->setID(dataRobot->getObject("item")->get("id").convert<std::string>());
                robotStatus->setToken(dataRobot->getObject("item")->get("token").convert<std::string>());
                robotStatus->setEnable(dataRobot->getObject("item")->get("is_enabled").convert<bool>());
                robotStatus->setActive(dataRobot->getObject("item")->get("job_is_active").convert<bool>());
                robotInfo->setID(dataRobot->getObject("item")->get("id").convert<std::string>());

                NcAgentConfig::get().setRobotID(robotStatus->getID());
                NcAgentConfig::get().setToken(robotStatus->getToken());
            }
            else {
                LOG_ERROR("Robot register error status error : Status %d Reason %s", (int)status, reason.c_str());
            }
        }
        else {
            LOG_WARNING("Robot information has been deleted.");
            res = Net::HTTP::get(Poco::format("/robots/find-amr/%s", ipAddress));
            body = std::get<0>(res);
            status = std::get<1>(res);
            reason = std::get<2>(res);

            if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                Poco::JSON::Object::Ptr savedRobot = JSONParse(body);
                auto item = savedRobot->getObject("item");

                std::string id = item->get("id").convert<std::string>();
                std::string token = item->get("token").convert<std::string>();
                bool isEnabled = item->get("is_enabled").convert<bool>();
                bool isActive = item->get("job_is_active").convert<bool>();

                robotStatus->setID(id);
                robotStatus->setToken(token);
                robotStatus->setEnable(isEnabled);
                robotStatus->setActive(isActive);
                robotInfo->setID(id);

                NcAgentConfig::get().setRobotID(id);
                NcAgentConfig::get().setToken(token);
            }
            else {
                LOG_ERROR("Robot register error status error : Status %d Reason %s", (int)status, reason.c_str());
                return;  // 초기화 실패 → 그냥 종료
            }
        }
    }
    else if (
        status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_BAD_REQUEST ||
        status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_METHOD_NOT_ALLOWED) {
        LOG_ERROR("backend /robots/create status error : Status %d Reason %s", (int)status, reason.c_str());
        return;
    }
    else {
        LOG_ERROR("backend /robots/create status error : Status %d Reason %s", (int)status, reason.c_str());
        return;
    }

    // robot_id Config에 저장
    Config::instance().setString("robot_id", robotStatus->getID());

    LOG_INFO("Robot verification & registration completed.");
}

}  // namespace NaviFra
