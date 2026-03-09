#pragma once

#include "action_param.h"
#include "Handler/ActionHandler.h"
#include "Handler/spinActionHandler.h"

#include <Poco/JSON/Array.h>

#include <map>
#include <memory>
#include <string>

namespace NaviFra {

class TaskBuilder {
public:
    TaskBuilder(const std::string& uuid);

    void processWork(Poco::JSON::Object::Ptr work, std::shared_ptr<BaseParams> next_params = nullptr, int next_action_type = -1);

    Poco::JSON::Array::Ptr getResult();

    const ActionData& getActionData() const;
    bool isSpinForDocking(Poco::JSON::Object::Ptr work);
    void prepareEntryForAction(Poco::JSON::Object::Ptr work, ActionHandler* handler);

private:
    std::string uuid_;
    ActionData action_data_;
    Poco::JSON::Object::Ptr current_entry_;
    Poco::JSON::Array::Ptr data_array_;
    int prev_action_type_ = -1;

    std::map<int, std::unique_ptr<ActionHandler>> action_handlers_;

    std::unique_ptr<SpinActionHandler> spin_handler_;
};
}  // namespace NaviFra