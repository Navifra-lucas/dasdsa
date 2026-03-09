#ifndef NAVIFRA_ACTION_START_SCAN_H
#define NAVIFRA_ACTION_START_SCAN_H

#include "core_astra/action/action_base.h"

#include <ros/publisher.h>

namespace NaviFra {
class ActionStartSCAN : public ActionBase {
public:
    ActionStartSCAN();
    virtual ~ActionStartSCAN();

private:
    // 실행 메서드 (필수)
    void handle(ActionContext& ctx, Poco::JSON::Object::Ptr req) override;

    // 액션 이름
    std::string name() const override { return "start_scan"; }
};

REGISTER_ACTION("start_scan", ActionStartSCAN, ActionType::DEFAULT)
}  // namespace NaviFra
#endif