#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_robot_list.h>

using namespace NaviFra;

NcActionRobotList::NcActionRobotList()
{
    v2v_pub_ = nh.advertise<core_msgs::VehicleList>("/v2v_info", 10);
}

NcActionRobotList::~NcActionRobotList()
{
}

std::string NcActionRobotList::implName()
{
    return "NcActionRobotList";
}

void NcActionRobotList::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    // [0] : robot x
    // [1] : robot y
    // [2] : robot theta
    // [3] : robot loaded
    // [4] : 차상 길이(앞뒤)
    // [5] : 차상 길이(좌우)
    // [6] : 로봇 직선 속도
    // [7] : 로봇 회전 속도

    // 이닛포즈 시 위치잡는 범위 확인

    try {

        core_msgs::VehicleList vehicle_list;

        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");

        Poco::SharedPtr<Poco::JSON::Array> robots = obj->getArray("msg");
        for (size_t i = 0; i < robots->size(); i++) {
            Poco::SharedPtr<Poco::JSON::Object> robot = robots->getObject(i);

            Poco::SharedPtr<Poco::JSON::Object> layout = robot->getObject("layout");
            Poco::SharedPtr<Poco::JSON::Object> dim = layout->getObject("dim");
            std::string label = dim->getValue<std::string>("label");

            Poco::JSON::Array::Ptr data = robot->getArray("data");

            float r_v = data->getElement<float>(6);
            float r_theta = data->getElement<float>(2);
            float vx = r_v * std::cos(r_theta);
            float vy = r_v * std::sin(r_theta);

            core_msgs::Vehicle vehicle;
            vehicle.id = label;
            vehicle.x_m = data->getElement<float>(0);
            vehicle.y_m = data->getElement<float>(1);
            vehicle.angle_deg = r_theta * 180 / M_PI;
            vehicle.f_linear_speed_x_ms = r_v;
            vehicle.f_linear_speed_y_ms = 0;
            vehicle.f_angular_speed_z_degs = data->getElement<float>(7);
            vehicle.f_robot_size_front_m = data->getElement<float>(4) / 2;
            vehicle.f_robot_size_rear_m = data->getElement<float>(4) / 2;
            vehicle.f_robot_size_left_m = data->getElement<float>(5) / 2;
            vehicle.f_robot_size_right_m = data->getElement<float>(5) / 2;

            vehicle_list.data.push_back(vehicle);
        }

        v2v_pub_.publish(vehicle_list);

        auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        Poco::Timestamp now;
        robotStatus->setLastSubTime(now);

        // 응답 메시지 전송
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "robot_list");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);
        
        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "robotList Exception: " << ex.displayText();
    }
}
