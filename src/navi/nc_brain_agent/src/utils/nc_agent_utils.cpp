#include "core_agent/data/memory_repository.h"

#include <core_agent/data/robot_status.h>
#include <core_agent/message/message_publisher.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>
#include <sys/reboot.h>
#include <unistd.h>

using Poco::File;
using Poco::JSON::Object;

namespace NaviFra {
bool isWiFiInterface(const Poco::Net::NetworkInterface& iface)
{
    // 인터페이스 이름을 기반으로 Wi-Fi 인터페이스 여부를 판단
    std::string displayName = iface.displayName();
    return (displayName.find("wlan") != std::string::npos) || (displayName.find("wifi") != std::string::npos) ||
        (displayName.find("wl") != std::string::npos);
}

std::string getLocalAddress()
{
    Poco::Net::NetworkInterface::List interfaces = Poco::Net::NetworkInterface::list();

    for (const auto& iface : interfaces) {
        if (iface.supportsIPv4() && !iface.isLoopback() && isWiFiInterface(iface)) {
            const Poco::Net::IPAddress& ip = iface.address();
            return ip.toString();
        }
    }

    return "127.0.0.1";
}

Poco::JSON::Object::Ptr JSONParse(std::string str)
{
    assert(!str.empty());

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(str);
    Poco::JSON::Object::Ptr data = result.extract<Poco::JSON::Object::Ptr>();

    return std::move(data);
}

Poco::JSON::Object::Ptr loadJSON(std::string filename)
{
    if (File(filename).exists()) {
        std::ostringstream ostr;
        Poco::FileInputStream fis(filename);
        Poco::StreamCopier::copyStream(fis, ostr);
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(ostr.str());
        Poco::JSON::Object::Ptr data = result.extract<Poco::JSON::Object::Ptr>();

        return std::move(data);
    }
    return nullptr;
}

void saveJSON(std::string path, Object::Ptr obj)
{
    try {
        if (File(path).exists()) {
            Poco::LocalDateTime now;
            std::string backupPath = Poco::format("%s/navifra_solution/navicore/configs/backup/", std::string(std::getenv("HOME")));
            std::string backupfilepath = Poco::format(
                "%s%s_%s", backupPath, Poco::DateTimeFormatter::format(now.timestamp(), "%Y%m%d%H%M%S"), Poco::Path(path).getFileName());
            if (!Poco::File(backupPath).exists())
                File(backupPath).createDirectories();
            File(path).copyTo(backupfilepath);
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what() << '\n';
    }
    std::ostringstream oss;
    obj->stringify(oss, 4);
    std::string jsonString = oss.str();
    Poco::FileOutputStream fileStream(path);
    fileStream << jsonString;
    fileStream.close();
}

void saveJSON(std::string path, Object obj)
{
    try {
        if (File(path).exists()) {
            Poco::LocalDateTime now;
            std::string backupPath = Poco::format("%s/navifra_solution/navicore/configs/backup/", std::string(std::getenv("HOME")));
            std::string backupfilepath = Poco::format(
                "%s%s_%s", backupPath, Poco::DateTimeFormatter::format(now.timestamp(), "%Y%m%d%H%M%S"), Poco::Path(path).getFileName());
            if (!Poco::File(backupPath).exists())
                File(backupPath).createDirectories();
            File(path).copyTo(backupfilepath);
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what() << '\n';
    }

    std::ostringstream oss;
    obj.stringify(oss, 4);
    std::string jsonString = oss.str();
    Poco::FileOutputStream fileStream(path);
    fileStream << jsonString;
    fileStream.close();
}

void robotreboot()
{
#ifdef __unix__  // linux 일때 만
    sync();
    int result = std::system("sudo reboot now");
#endif
}

void sendResponseSuccess(std::string source, std::string uuid, std::string reslut, std::string msg)
{
    try {
        Object response, emptyData;
        response.set("uuid", uuid);
        response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
        response.set("result", reslut);
        response.set("message", msg);
        response.set("data", emptyData);

        std::ostringstream oss;
        response.stringify(oss);
        MessageBroker::instance().publish(
            source, NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
            oss.str());
    }
    catch (Poco::Exception ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}

void sendResponseSuccessWithData(std::string source, std::string uuid, Poco::JSON::Object::Ptr data, std::string reslut)
{
    try {
        Object response;
        response.set("uuid", uuid);
        response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
        response.set("result", reslut);
        response.set("data", data);

        std::ostringstream oss;
        response.stringify(oss);
        MessageBroker::instance().publish(
            source, NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
            oss.str());
    }
    catch (Poco::Exception ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}

void sendResponseSuccessWithData(std::string source, std::string uuid, Poco::JSON::Object data, std::string reslut)
{
    try {
        Object response;
        response.set("uuid", uuid);
        response.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
        response.set("result", reslut);
        response.set("data", data);

        std::ostringstream oss;
        response.stringify(oss);
        MessageBroker::instance().publish(
            source, NcBrainMessage::MESSAGE_ROBOT_RESPONSE + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
            oss.str());
    }
    catch (Poco::Exception ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}
}  // namespace NaviFra