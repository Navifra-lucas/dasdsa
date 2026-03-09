#ifndef NB_AGENT_UTILS_H
#define NB_AGENT_UTILS_H

#include <Poco/JSON/Object.h>
#include <Poco/Net/IPAddress.h>
#include <Poco/Net/NetworkInterface.h>

namespace NaviFra {
bool isWiFiInterface(const Poco::Net::NetworkInterface& iface);
std::string getLocalAddress();
Poco::JSON::Object::Ptr JSONParse(std::string str);
Poco::JSON::Object::Ptr loadJSON(std::string filename);
void saveJSON(std::string path, Poco::JSON::Object::Ptr obj);
void saveJSON(std::string path, Poco::JSON::Object obj);
void robotreboot();
void sendResponseSuccess(std::string source, std::string uuid, std::string reslut = "success", std::string msg = "");
void sendResponseSuccessWithData(std::string source, std::string uuid, Poco::JSON::Object::Ptr data, std::string reslut = "success");
void sendResponseSuccessWithData(std::string source, std::string uuid, Poco::JSON::Object data, std::string reslut = "success");
}  // namespace NaviFra

#endif  // NB_AGENT_UTILS_H
