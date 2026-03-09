#ifndef NC_AGENT_PARAMETERS_H
#define NC_AGENT_PARAMETERS_H

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/Timestamp.h>
#include <yaml-cpp/yaml.h>

namespace NaviFra {
class NcAgentParameters {
public:
    NcAgentParameters();
    ~NcAgentParameters();

    const static std::string KEY;

    static NcAgentParameters& get();

public:
    YAML::Node getParameters();
    YAML::Node getParametersRobot();
    void setParameters(std::string key, float value);
    Poco::JSON::Object toObject();
    void updateParameters(Poco::JSON::Object::Ptr data);
    bool isExits();

private:
    Poco::JSON::Array parserSequencetoJSON(YAML::Node node);
    Poco::JSON::Object parserMaptoJSON(YAML::Node node);
    std::string typeName(std::string node);

    YAML::Node parserSequencetoYAML(Poco::JSON::Array obj);
    YAML::Node parserMaptoYAML(Poco::JSON::Object obj);
    virtual std::string implName() { return "NcAgentParameters"; }

private:
    std::string default_dirpath_;
    std::string dirpath_;
    std::string default_filepath_;
    std::string filepath_;
};
}  // namespace NaviFra

#endif  // NC_AGENT_PARAMETERS_H