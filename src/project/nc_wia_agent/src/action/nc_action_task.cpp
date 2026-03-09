#include "nc_wia_agent/action/nc_action_task.h"

namespace NaviFra {

NcActionTask::NcActionTask()
{
}

NcActionTask::~NcActionTask()
{
}

std::string NcActionTask::implName()
{
    return "NcActionTask";
}

void NcActionTask::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");
        Poco::JSON::Object::Ptr goal_id = obj->getObject("goal_id");
        std::string uuid = goal_id->getValue<std::string>("id");

        Poco::JSON::Object::Ptr goal_obj = obj->getObject("goal");
        int loop_flag = goal_obj->getValue<int>("loop_flag");
        int start_idx = goal_obj->getValue<int>("start_idx");
        std::string task_id = goal_obj->getValue<std::string>("task_id");
        Poco::JSON::Array::Ptr missionlist = goal_obj->getArray("missionlist");

        TaskResultPublisher::instance().setTaskContext(rid, task_id, msg_id);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "task");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());

        auto battery_status = InMemoryRepository::instance().get<BatteryStatus>(BatteryStatus::KEY);
        auto f_curr_battery = battery_status->getCapacity();

        if (f_curr_battery <= 20.0) {
            NLOG(info) << "Battery Capacity is lower then 20%";
            TaskResultPublisher::instance().publish("REJECTED");
            return;
        }

        // 4. TaskBuilder를 사용하여 작업 목록 생성
        TaskBuilder builder(uuid);

        for (size_t loop = 0; loop < loop_flag; loop++) {
            for (size_t i = start_idx; i < missionlist->size(); ++i) {
                Poco::JSON::Object::Ptr mission = missionlist->getObject(i);
                Poco::JSON::Array::Ptr work_array = mission->getArray("work");
                size_t action_start_idx = mission->getValue<int>("action_start_idx");

                for (size_t j = action_start_idx; j < work_array->size(); ++j) {
                    std::shared_ptr<BaseParams> next_params = nullptr;
                    int n_next_action_type = -1;
                    
                    if (j + 1 < work_array->size()) {
                        Poco::JSON::Object::Ptr next_work = work_array->getObject(j + 1);
                        n_next_action_type = next_work->getValue<int>("action_type");
                        
                        if (n_next_action_type == ACTION_FORK) {
                            next_params = std::make_shared<ForkLiftParams>(ForkLiftParams::from(next_work));
                        }
                        else if (n_next_action_type == ACTION_MOVE) {
                            next_params = std::make_shared<MoveParams>(MoveParams::from(next_work));
                        }
                    }
                    builder.processWork(work_array->getObject(j), next_params, n_next_action_type);
                }
            }
        }

        Poco::JSON::Object::Ptr final_task_json = new Poco::JSON::Object;
        final_task_json->set("action", "add_task");
        final_task_json->set("uuid", uuid);
        final_task_json->set("data", builder.getResult());

        std::ostringstream oss_task;
        final_task_json->stringify(oss_task);
        std::string final_json_str = oss_task.str();
        NLOG(info) << final_json_str;

        auto srvCallReq = requestStringSrv("add_task", final_json_str);
        NLOG(info) << "service request result : " << srvCallReq.success;

        if (srvCallReq.success) {
            auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
            if (status) {
                const auto& action_data = builder.getActionData();
                status->setMsgID(msg_id);
                status->setTaskID(task_id);
                status->setLoopCnt(loop_flag);
                status->setGoalInfo(action_data.goal_info);
                saveTaskTxt(final_json_str);
            }
        }
        else {
            TaskResultPublisher::instance().publish("REJECTED");
        }
    }
    catch (const Poco::BadCastException& e) {
        NLOG(error) << "[BadCastException] " << e.displayText();
        throw;
    }
    catch (const Poco::Exception& e) {
        NLOG(error) << "[PocoException] " << e.displayText();
        throw;
    }
    catch (const std::exception& e) {
        NLOG(error) << "[std::exception] " << e.what();
        throw;
    }
    catch (...) {
        NLOG(error) << "[UnknownException]";
        throw;
    }
}

}  // namespace NaviFra