#include "gtest/gtest.h"
#include "nc_task_manager/nc_task_manager_pch.h"

#include <boost/sml.hpp>
#include <nc_task_manager/data/nc_task.h>
#include <nc_task_manager/state/nc_state_move.h>

#include <chrono>
#include <thread>

namespace sml = boost::sml;
namespace NaviFra {

class StateMachineFixture : public ::testing::Test {
protected:
    void SetUp() override
    {
        int argc = 0;
        char** argv;

        ros::init(argc, argv, "nc_task_manager");
    }

    void TearDown() override { ros::shutdown(); }
};

TEST_F(StateMachineFixture, normal)
{
    ros::NodeHandle nh_;
    NcStateMove moveState(nh_);
    EXPECT_EQ(moveState.getCurrentState(), 0);

    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    obj->set("start_node", "0001");
    obj->set("end_node", "0002");
    obj->set("finish_angle", 999);
    obj->set("is_align", false);
    obj->set("type", "move");
    obj->set("uuid", "2b72c363-dc9e-4e9f-b1cc-06f34b45f0e3");

    Task task(obj);
    AddGoalTask addgoal;
    addgoal.tasks.push_back(task);
    moveState.AddGoal(std::move(&addgoal));

    // sm.process_event(NaviFra::Events::onTaskAdd{NaviFra::Task{task}});
    EXPECT_EQ(moveState.getCurrentState(), NcStateMove::ST_RUN);

    ArrivedGoal arrivedgoal;
    arrivedgoal.uuid = "2b72c363-dc9e-4e9f-b1cc-06f34b45f0e3";
    moveState.ArrivedGoals(&arrivedgoal);

    EXPECT_EQ(moveState.getCurrentState(), NcStateMove::ST_IDLE);

    // task.s_uuid = "9b8b1bc1-7752-4a68-83c5-1c6752a82159";
    // task.s_type = TYPE.MOVE;
    // task.s_start_node_name = "0002";
    // task.s_end_node_name = "0003";
    // task.f_align_angle_deg = 999;
    // task.b_align = false;

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // sm.process_event(NaviFra::Events::onTaskAdd{NaviFra::Task{task}});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::run>));

    // task.s_uuid = "2f63e443-46f0-494d-aab2-39956b9a38f6";
    // task.s_type = TYPE.TURN;
    // task.s_start_node_name = "0003";
    // task.s_end_node_name = "0003";
    // task.f_align_angle_deg = 999;
    // task.b_align = false;

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // sm.process_event(NaviFra::Events::onTaskAdd{NaviFra::Task{task}});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::run>));

    // task.s_uuid = "8f67f430-9a3c-4e84-b8b6-7cd04cfcc413";
    // task.s_type = TYPE.MOVE;
    // task.s_start_node_name = "0003";
    // task.s_end_node_name = "0004";
    // task.f_align_angle_deg = 999;
    // task.b_align = false;

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // sm.process_event(NaviFra::Events::onTaskAdd{NaviFra::Task{task}});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::run>));

    // sm.process_event(NaviFra::Events::onTaskDone{manager.currentTask().value()});
    // sm.process_event(NaviFra::Events::onTaskDone{manager.currentTask().value()});
    // sm.process_event(NaviFra::Events::onTaskDone{manager.currentTask().value()});
    // sm.process_event(NaviFra::Events::onTaskDone{manager.currentTask().value()});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::idle>));
    // EXPECT_FALSE(manager.currentTask().has_value());
}

TEST_F(StateMachineFixture, cancle)
{
    // Task task;
    // TaskManager manager;
    // StateLogger logger;
    // sml::sm<TaskManagerSM, StateLogger> sm{manager, logger};
    // EXPECT_FALSE(manager.currentTask().has_value());
    //
    // task.s_uuid = "2b72c363-dc9e-4e9f-b1cc-06f34b45f0e3";
    // task.s_type = TYPE.MOVE;
    // task.s_start_node_name = "0001";
    // task.s_end_node_name = "0002";
    // task.f_align_angle_deg = 999;
    // task.b_align = false;
    //
    // sm.process_event(NaviFra::Events::onTaskAdd{task});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::run>));
    //
    // sm.process_event(NaviFra::Events::onCancel{});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::idle>));
    // EXPECT_FALSE(manager.currentTask().has_value());
    // EXPECT_EQ(manager.taskSize(), 0);
}

TEST_F(StateMachineFixture, resume)
{
    // Task task;
    // TaskManager manager;
    // StateLogger logger;
    // sml::sm<TaskManagerSM, StateLogger> sm{manager, logger};
    // EXPECT_FALSE(manager.currentTask().has_value());
    //
    // task.s_uuid = "2b72c363-dc9e-4e9f-b1cc-06f34b45f0e3";
    // task.s_type = TYPE.MOVE;
    // task.s_start_node_name = "0001";
    // task.s_end_node_name = "0002";
    // task.f_align_angle_deg = 999;
    // task.b_align = false;
    //
    // sm.process_event(NaviFra::Events::onTaskAdd{task});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::run>));
    //
    // sm.process_event(NaviFra::Events::onPause{});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::pause>));
    //
    // sm.process_event(NaviFra::Events::onResume{});
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::run>));
    //
    // sm.process_event(NaviFra::Events::onTaskDone{manager.currentTask().value()});
    //
    // EXPECT_TRUE(sm.is(sml::state<NaviFra::idle>));
    // EXPECT_FALSE(manager.currentTask().has_value());
    // EXPECT_EQ(manager.taskSize(), 0);
}

TEST_F(StateMachineFixture, normalwithtopic)
{
    // TaskManager manager;
    // ros::NodeHandle nh;
    // ros::Publisher task_add = nh.advertise<core_msgs::AddTask>("/nc_task_manager/task_add", 1);
    // EXPECT_FALSE(manager.currentTask().has_value());
    //
    // core_msgs::AddTask taskMSG;
    // taskMSG.uuid = "2b72c363-dc9e-4e9f-b1cc-06f34b45f0e3";
    // taskMSG.type = TYPE.MOVE;
    // taskMSG.start_node_name = "0001";
    // taskMSG.end_node_name = "0002";
    // taskMSG.finish_angle = 999;
    // taskMSG.is_align = false;
    //
    // task_add.publish(taskMSG);
    //
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));  // publish 하고 너무 빨리 체크 하면 메세지 받지 못함
    //
    // ros::spinOnce();
    // EXPECT_TRUE(manager.isState(sml::state<NaviFra::run>));
    //
    // ros::Publisher info = nh.advertise<core_msgs::NavicoreStatus>("/navifra/info", 1);
    //
    // core_msgs::NavicoreStatus status;
    // status.s_status = STATUS.RUNNING;
    // status.s_current_node_id = "0001";
    // status.s_current_node = "0001";
    // status.s_next_node_id = "0002";
    // status.s_next_node = "0002";
    // info.publish(status);
    //
    // status.s_status = STATUS.RUNNING;
    // status.s_current_node_id = "0002";
    // status.s_current_node = "0002";
    // status.s_next_node_id = "0003";
    // status.s_next_node = "0003";
    // info.publish(status);
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));  // publish 하고 너무 빨리 체크 하면 메세지 받지 못함
    // ros::spinOnce();

    // EXPECT_TRUE(manager.isState(sml::state<NaviFra::idle>));
}

}  // namespace NaviFra