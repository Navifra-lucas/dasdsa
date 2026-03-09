#ifndef NC_ACTION_TASK_H
#define NC_ACTION_TASK_H

#include "nc_wia_agent/data/battery_status.h"
#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/util/task_builder/task_builder.h"
#include "nc_wia_agent/util/task_memory_util.h"
#include "nc_wia_agent/util/task_result_publisher.h"

#include <Poco/DateTimeFormatter.h>
#include <Poco/LocalDateTime.h>
#include <core_agent/action/action_base.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/memory_repository.h>

#include <stdexcept>
namespace NaviFra {

class NcActionTask : public ActionBase {
public:
    NcActionTask();
    virtual ~NcActionTask();

protected:
    virtual void implonAction(std::string source, Poco::JSON::Object::Ptr obj) override;
    std::string implName() override;
};

REGISTER_ACTION("task", NcActionTask, ActionType::DEFAULT)
}  // namespace NaviFra
#endif  // NC_ACTION_AMR_BASIC_H