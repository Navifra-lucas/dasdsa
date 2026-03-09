#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/utils/nc_agent_config.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>

using Poco::File;
using Poco::JSON::Array;
using Poco::JSON::Object;

using namespace NaviFra;

namespace {
static Poco::SingletonHolder<NcAgentConfig> sh;
}

NcAgentConfig& NcAgentConfig::get()
{
    return *sh.get();
}

NcAgentConfig::NcAgentConfig()
{
    try {
        /*
            현재 설정 파일은 환경 변수와 JSON 파일 두개로 나뉘어져 있는데 둘다 설정을 하더라도 입력 할 수 있게 변경 되어야 한다.
            일단 Config Class를 MapConfiguration을 상속 받아 파일에 자유롭게 쓸 수 있게 구현 해 놓고
            향후에 모든 설정 파일을 통합 한 후에 설정 옵션을 가져가는 것으로 하자
        */
        const char* HOME_DIR = std::getenv("HOME");
        auto s_robot_ip = std::getenv("ROBOT_IP");
        std::string homeDIR = (HOME_DIR != NULL) ? HOME_DIR : "/root";
        std::string extention = ".json";
        std::string default_file_name = "nc_brain_agent_develop.json";
        filepath_ = homeDIR + "/navifra_solution/navicore/configs/configs/";
        
        try {
            // Poco::File 사용하여 디렉토리 확인 및 생성
            Poco::File configDir(filepath_);
            if (!configDir.exists()) {
                configDir.createDirectories();  // 중간 경로까지 생성
                std::cout << "Created directory: " << filepath_ << std::endl;
            } else if (!configDir.isDirectory()) {
                std::cerr << "Path exists but is not a directory: " << filepath_ << std::endl;
            }
        } catch (Poco::Exception& ex) {
            std::cerr << "Failed to create directory " << filepath_ << ": "
                      << ex.displayText() << std::endl;
        }

        filename_ = filepath_ + "nc_brain_agent_develop" + "_" + (s_robot_ip ? s_robot_ip : "localhost") + extention;

        // if (File(filepath_ + default_file_name).exists()) {
        //     try {
        //         if (File(filename_).exists()) {
        //             // File(filename_).copyTo(filename_ + ".bak");
        //             // File(filename_).remove();
        //             NLOG(info) << "ip file exist";
        //         }
        //         File(filepath_ + default_file_name).copyTo(filename_);
        //         NLOG(info) << "default file already exist.. copy file";
        //     }
        //     catch (const std::exception& e) {
        //         NLOG(info) << "cannot copy";
        //     }
        // }

        if (!File(filepath_).exists()) {
            File(filepath_).createDirectories();
            LOG_INFO("Agent - Config folder connected");
        }

        if (!File(filename_).exists()) {
            std::string launch_path = ros::package::getPath("nc_brain_agent");
            if (launch_path.empty()) {
                launch_path = PAKAGE_FILE_PATH;
                LOG_INFO("Agent - Config file exist");
            }

            std::string config_path = launch_path + "/launch/configs/nc_brain_agent_config_exm.json";
            File(config_path).copyTo(filename_);

            LOG_INFO("Agent - Config file exist -> copy To nc_brain_agent_config_exm.json");
        }
        // }

        // Poco::Util::AbstractConfiguration::Keys k;
        // keys(k);

        // NLOG(info) << "The set config is as follows.";
        // for (const auto& key : k) {
        //    NLOG(info) << key << " = " << getString(key);
        //}
    }
    catch (std::exception& ex) {
        NLOG(error) << ex.what();
    }
}
NcAgentConfig::~NcAgentConfig()
{
}

void NcAgentConfig::setRobotID(std::string id)
{
    Object::Ptr config = loadJSON(filename_);
    config->getObject("env")->set("id", id);
    setString("robot_id", id);
    saveJSON(filename_, config);
}

void NcAgentConfig::setToken(std::string client_token)
{
    Object::Ptr config = loadJSON(filename_);
    config->getObject("env")->set("client_token", client_token);
    saveJSON(filename_, config);
}

std::string NcAgentConfig::getRobotID()
{
    Object::Ptr config = loadJSON(filename_);
    return config->getObject("env")->get("id").convert<std::string>();
}

std::string NcAgentConfig::getToken()
{
    Object::Ptr config = loadJSON(filename_);
    return config->getObject("env")->get("client_token").convert<std::string>();
}

std::string NcAgentConfig::getModelType_cd()
{
    Object::Ptr config = loadJSON(filename_);
    LOG_INFO("Agent - Config getModelType_cd() : In");
    if (config->getObject("env")->isNull("model_type_cd")) {
        LOG_INFO("Agent - Config getModelType_cd() : Out default Value");
        return "D10001";
    }
    else {
        LOG_INFO("Agent - Config getModelType_cd() : Out set Value");
        return config->getObject("env")->get("model_type_cd").convert<std::string>();
    }
}

std::string NcAgentConfig::getDrivingType_cd()
{
    Object::Ptr config = loadJSON(filename_);
    LOG_INFO("Agent - Config getDrivingType_cd() : In");
    if (config->getObject("env")->isNull("driving_type_cd")) {
        LOG_INFO("Agent - Config getDrivingType_cd() : Out default Value");
        return "D30001";
    }
    else {
        LOG_INFO("Agent - Config getDrivingType_cd() : Out set Value");
        return config->getObject("env")->get("driving_type_cd").convert<std::string>();
    }
}

std::string NcAgentConfig::getRobotBasePos()
{
    Object::Ptr config = loadJSON(filename_);
    if (config->getObject("env")->isNull("robot_base_pos")) {
        return "driver_wheel/f_FL_wheel_x_m";
    }
    else {
        return config->getObject("env")->get("robot_base_pos").convert<std::string>();
    }
}

std::string NcAgentConfig::getRobotBaseOffset()
{
    Object::Ptr config = loadJSON(filename_);
    if (config->getObject("env")->isNull("robot_base_offset")) {
        return "driver_wheel/f_FL_wheel_y_m";
    }
    else {
        return config->getObject("env")->get("robot_base_offset").convert<std::string>();
    }
}

void NcAgentConfig::initializeRobot()
{
    Object::Ptr config = loadJSON(filename_);
    config->getObject("env")->set("uuid", "");
    config->getObject("env")->set("id", "");
    config->getObject("env")->set("client_token", "");
    config->getObject("env")->set("ip_address", "");
    saveJSON(filename_, config);
}

void NcAgentConfig::validCheck(std::string id, std::string token)
{
    Object::Ptr config = loadJSON(filename_);

    if (config->getObject("env")->get("id").convert<std::string>().compare(id) != 0 ||
        config->getObject("env")->get("client_token").convert<std::string>().compare(token)) {
        config->getObject("env")->set("id", id);
        config->getObject("env")->set("client_token", token);
    }
}

Poco::JSON::Array::Ptr NcAgentConfig::getCtrItems()
{
    Object::Ptr config = loadJSON(filename_);
    return config->getObject("ros")->getArray("control");
}