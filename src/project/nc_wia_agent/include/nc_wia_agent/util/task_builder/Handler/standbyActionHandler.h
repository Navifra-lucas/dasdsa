#pragma once
#include "ActionHandler.h"
#include "nc_wia_agent/util/task_builder/common_types.h"

namespace NaviFra {

class StandbyActionHandler : public ActionHandler {
public:
    StandbyActionHandler();
    ~StandbyActionHandler() override;

    void process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data) override;
    std::string getTypeName(Poco::JSON::Object::Ptr work) const override;

private:
};
}  // namespace NaviFra