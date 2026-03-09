#ifndef NC_AGENT_NODE_MANAGER_H
#define NC_AGENT_NODE_MANAGER_H

#include <Poco/JSON/Object.h>

namespace NaviFra {
class NcAgentNodeManager {
public:
    NcAgentNodeManager();
    ~NcAgentNodeManager();

    static NcAgentNodeManager& get();

public:
    void updateNodes();
    std::string toString();
    Poco::JSON::Object::Ptr getNodeStatus();

private:
    Poco::JSON::Object::Ptr nodes_;
};
}  // namespace NaviFra

#endif  // NC_AGENT_NODE_MANAGER_H