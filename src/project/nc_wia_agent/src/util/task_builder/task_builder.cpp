#include "nc_wia_agent/util/task_builder/task_builder.h"

#include "nc_wia_agent/util/task_builder/Handler/chargeActionHandler.h"
#include "nc_wia_agent/util/task_builder/Handler/dockingActionHandler.h"
#include "nc_wia_agent/util/task_builder/Handler/forkliftActionHandler.h"
#include "nc_wia_agent/util/task_builder/Handler/liftActionHandler.h"
#include "nc_wia_agent/util/task_builder/Handler/moveActionHandler.h"
#include "nc_wia_agent/util/task_builder/Handler/standbyActionHandler.h"
#include "nc_wia_agent/util/task_builder/Handler/undockingActionHandler.h"

#define TESTMAX 100000

using namespace NaviFra;

TaskBuilder::TaskBuilder(const std::string& uuid)
    : uuid_(uuid)
{
    data_array_ = new Poco::JSON::Array;
    action_data_.o_last_pos.SetXm(TESTMAX);
    action_data_.o_last_pos.SetYm(TESTMAX);

    action_handlers_[ACTION_MOVE] = std::make_unique<MoveActionHandler>();
    action_handlers_[ACTION_DOCKING] = std::make_unique<DockingActionHandler>();
    action_handlers_[ACTION_STANDBY] = std::make_unique<StandbyActionHandler>();
    action_handlers_[ACTION_LIFT] = std::make_unique<LiftActionHandler>();
    action_handlers_[ACTION_DOCKING_OUT] = std::make_unique<UndockingActionHandler>();
    action_handlers_[ACTION_FORK] = std::make_unique<ForkliftActionHandler>();
    action_handlers_[ACTION_CHARGE] = std::make_unique<ChargeActionHandler>();
    action_handlers_[ACTION_WINGBODY_PERCEPTION] = std::make_unique<ForkliftActionHandler>();

    spin_handler_ = std::make_unique<SpinActionHandler>();
}

void TaskBuilder::prepareEntryForAction(Poco::JSON::Object::Ptr work, ActionHandler* handler)
{
    if (!current_entry_.isNull()) {
        data_array_->add(current_entry_);
    }

    current_entry_ = new Poco::JSON::Object;
    current_entry_->set("uuid", uuid_);

    current_entry_->set("type", handler->getTypeName(work));
}

bool TaskBuilder::isSpinForDocking(Poco::JSON::Object::Ptr work)
{
    Poco::JSON::Array::Ptr args = work->getArray("action_args");

    MoveDirection dock_direction = (args->getElement<int>(0) == 1) ? MOVE_FORWARD : MOVE_BACKWARD;

    return dock_direction != action_data_.last_drive_direction;
}

void TaskBuilder::processWork(Poco::JSON::Object::Ptr work, std::shared_ptr<BaseParams> next_params, int next_action_type)
{
    int n_curr_action_type = work->getValue<int>("action_type");
    int n_curr_action_id = work->getValue<int>("action_id");
    NLOG(info) << "--------------------------------------";
    NLOG(info) << "action_id : " << n_curr_action_id << "/ action type : " << n_curr_action_type;

    // 다음 액션이 move면 goal_pos 저장
    if (next_action_type == ACTION_MOVE && next_params) {
        auto next_move_params = std::dynamic_pointer_cast<MoveParams>(next_params);
        if (next_move_params) {
            action_data_.o_next_pos = next_move_params->o_goal_pos;
            action_data_.b_has_next_goal = true;
        }
        else {
            action_data_.b_has_next_goal = false;
        }
    }
    else {
        action_data_.b_has_next_goal = false;
    }


    // next_params가 존재하고, ForkLiftParams 타입인지 확인 및 변환
    if (next_params) {
        auto forklift_params = std::dynamic_pointer_cast<ForkLiftParams>(next_params);
        if (forklift_params) {
            // 변환 성공! 이제 ForkLiftParams 멤버에 접근 가능
            Poco::JSON::Array::Ptr args = work->getArray("action_args");

            if (args && args->size() == 3) {
                // 주의: getElement<int>는 소수점을 버릴 수 있으므로 float 사용 권장
                auto target_x = args->getElement<float>(0);
                auto target_y = args->getElement<float>(1);
                auto target_theta = args->getElement<float>(2);  // Radian 가정
                NLOG(info) << "Modified action_args before offset: " << target_x << ", " << target_y << ", " << target_theta * 180.0 / M_PI;

                // args->set(0, forklift_params->o_offset_pos.GetXm() + target_x);
                // args->set(1, forklift_params->o_offset_pos.GetYm() + target_y);
                // args->set(2, forklift_params->o_offset_pos.GetRad() + target_theta);

                // 명시적으로 work 객체에 변경된 배열을 반영 (포인터라 자동반영될 수 있으나 안전하게)
                work->set("action_args", args);
                target_x = args->getElement<float>(0);
                target_y = args->getElement<float>(1);
                target_theta = args->getElement<float>(2);  // Radian 가정

                NLOG(info) << "Modified action_args after offset: " << target_x << ", " << target_y << ", " << target_theta * 180.0 / M_PI;
            }
        }
    }

    if (n_curr_action_type == ACTION_DOCKING && isSpinForDocking(work)) {
        if (!current_entry_.isNull()) {
            data_array_->add(current_entry_);
        }

        current_entry_ = new Poco::JSON::Object;
        current_entry_->set("uuid", uuid_);

        //스핀턴 전용...
        current_entry_->set("type", spin_handler_->getTypeName(work));

        spin_handler_->process(work, current_entry_, action_data_);

        data_array_->add(current_entry_);
        current_entry_ = nullptr;  // 다음 작업(도킹)이 새 Entry를 만들도록 강제

        Poco::JSON::Array::Ptr args = work->getArray("action_args");
        action_data_.last_drive_direction = (args->getElement<int>(0) == 1) ? MOVE_FORWARD : MOVE_BACKWARD;
    }

    auto it = action_handlers_.find(n_curr_action_type);
    if (it == action_handlers_.end()) {
        return;  // 이다음에 task 오류 에러를 띄워야 하는데 어디서 띄울래....
    }

    ActionHandler* handler = it->second.get();

    if (current_entry_.isNull() || !(n_curr_action_type == ACTION_MOVE && prev_action_type_ == ACTION_MOVE)) {
        prepareEntryForAction(work, handler);
    }

    handler->process(work, current_entry_, action_data_);

    prev_action_type_ = n_curr_action_type;
}

Poco::JSON::Array::Ptr TaskBuilder::getResult()
{
    if (!current_entry_.isNull()) {
        data_array_->add(current_entry_);
        current_entry_ = nullptr;
    }
    return data_array_;
}

const ActionData& TaskBuilder::getActionData() const
{
    return action_data_;
}
