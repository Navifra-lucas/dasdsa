#ifndef NC_AGENT_CONFIG_H
#define NC_AGENT_CONFIG_H

#include <Poco/JSON/Object.h>
#include <Poco/Util/MapConfiguration.h>

namespace NaviFra {
class NcAgentConfig : public Poco::Util::MapConfiguration {
public:
    NcAgentConfig();
    ~NcAgentConfig();

    static NcAgentConfig& get();

public:
    void setRobotID(std::string id);
    void setToken(std::string client_token);

    std::string getModelType_cd();
    std::string getDrivingType_cd();
    std::string getRobotBasePos();
    std::string getRobotBaseOffset();

    std::string getRobotID();
    std::string getToken();
    void initializeRobot();
    void validCheck(std::string id, std::string token);
    Poco::JSON::Array::Ptr getCtrItems();

private:
    std::string filepath_;
    std::string filename_;
};
}  // namespace NaviFra

#endif  // NC_AGENT_CONFIG_H
