#pragma once
#include "ActionHandler.h"
#include "nc_wia_agent/util/task_builder/action_param.h"
#include "nc_wia_agent/util/task_builder/common_types.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace NaviFra {

class AddGoalActionHandler : public ActionHandler {
public:
    AddGoalActionHandler();
    ~AddGoalActionHandler() override;

    void process(Poco::JSON::Object::Ptr work, Poco::JSON::Object::Ptr& currentEntry, ActionData& action_data) override;
    std::string getTypeName(Poco::JSON::Object::Ptr work) const override;

private:
    double getYawFromOrientation(const Orientation& o);

    NaviFra::Pos calcPosfromPassingDist(Pos o_start_pos, Pos o_end_pos, float f_passing_dist);
    Poco::JSON::Object::Ptr addMoveData(MoveParams& action_params, Poco::JSON::Object::Ptr obj);
};
}  // namespace NaviFra