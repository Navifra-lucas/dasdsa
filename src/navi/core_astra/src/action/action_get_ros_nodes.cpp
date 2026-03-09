#include "core_astra/action/action_get_ros_nodes.h"

#include "core_agent/util/config.h"
#include "core_astra/util/sim_util.hpp"
#include "util/logger.hpp"

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <ros/master.h>

#include <chrono>
#include <cstdlib>
#include <random>
#include <thread>

namespace NaviFra {

void ActionGetROSNodes::handle(ActionContext& ctx, Poco::JSON::Object::Ptr /*req*/)
{
    LOG_INFO("[ActionGetROSNodes] 요청 처리 시작");

    // 취소 체크
    if (ctx.cancelled && *ctx.cancelled) {
        ctx.error(name(), ActionError::Cancelled, "Request cancelled");
        return;
    }

    if (simulate_delay_or_drop(ctx, name().c_str()))
        return;

    std::vector<std::string> rosNodes;
    ros::master::getNodes(rosNodes);

    std::string nodes_string =
        "{\"nodes\":{\"answer_core_bridge\":{\"description\":\"answer 컨트롤 노드\",\"can_control\":false},\"answer_node\":{\"description\":\"slam & localization 통합 노드\",\"can_control\":false},\"base_link_to_laser_link\":{\"description\":\"로봇중심과라이다중심의transform퍼블리시노드\",\"can_control\":false},\"laser_link_to_front_laser\":{\"description\":\"라이다중심과앞라이다중심의transform퍼블리시노드\",\"can_control\":false},\"laser_link_to_rear_laser\":{\"description\":\"라이다중심과뒤라이다중심의transform퍼블리시노드\",\"can_control\":false},\"launch_manager\":{\"description\":\"런치관리노드\",\"can_control\":false},\"nc_brain_map_server\":{\"description\":\"브레인UI를사용하는맵서버노드\",\"can_control\":false},\"nc_calibrator\":{\"description\":\"로봇의바퀴정보를캘리브레이션하는노드\",\"can_control\":false},\"nc_lidar_calibration\":{\"description\":\"라이다transform정보를캘리브레이션하는노드\",\"can_control\":false},\"nc_lidar_merger\":{\"description\":\"여러개의라이다정보를하나의정보로합치는노드\",\"can_control\":false},\"nc_navigator\":{\"description\":\"로봇의네비게이션노드\",\"can_control\":true},\"robot_state_pub\":{\"description\":\"로봇상태를퍼블리시하는노드\",\"can_control\":false},\"virtual_lidar_node_front\":{\"description\":\"가상의앞라이다정보퍼블리시노드\",\"can_control\":false},\"virtual_lidar_node_rear\":{\"description\":\"가상의뒤라이다정보퍼블리시노드\",\"can_control\":false}}}";

    std::string nodeDataPath = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/configs/ros_nodes.json";

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(nodes_string);
    Poco::JSON::Object::Ptr nodes = result.extract<Poco::JSON::Object::Ptr>();

    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    Poco::JSON::Array arr;

    Poco::JSON::Object::Ptr nodes_obj = nodes->getObject("nodes");

    auto found = std::string::npos;

    for (const std::string& name : nodes_obj->getNames()) {
        Poco::JSON::Object node;
        node.set("ros_node_id", name);
        node.set("description", nodes_obj->getObject(name)->get("description"));
        node.set("can_control", nodes_obj->getObject(name)->get("can_control"));

        found = std::string::npos;
        for (const std::string& current_node : rosNodes) {
            if (current_node.find(name) != std::string::npos) {
                found = 0;  // match
                break;
            }
        }
        node.set("running", found != std::string::npos);
        arr.add(node);
    }

    obj->set("list", arr);

    // 성공 응답
    ctx.ok(name(), obj);

    LOG_INFO("[ActionGetROSNodes] 응답 완료");
}

}  // namespace NaviFra
