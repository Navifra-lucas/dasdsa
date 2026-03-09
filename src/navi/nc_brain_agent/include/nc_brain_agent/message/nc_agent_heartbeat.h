#ifndef NC_AGENT_HEARTBEAT_STATUS_H
#define NC_AGENT_HEARTBEAT_STATUS_H

#include <Poco/Timestamp.h>

namespace NaviFra {
class NcAgentHearbeat {
public:
    NcAgentHearbeat();
    NcAgentHearbeat(std::string id, Poco::Timestamp time, bool is_slam);
    ~NcAgentHearbeat();

    using Ptr = std::shared_ptr<NcAgentHearbeat>;

private:
    std::string id_;
    Poco::Timestamp timestamp_;
    bool is_slam_;
    std::string version_;

public:
    void setID(std::string id);
    void setTimestamp(Poco::Timestamp time);
    void setIsSlam(bool is);
    void setVersion(const std::string& version);

    std::string toString();
};
}  // namespace NaviFra

#endif  // NC_AGENT_HEARTBEAT_STATUS_H