#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/data/nc_agent_node_manager.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>
#include <ros/master.h>

using namespace NaviFra;

namespace {
static Poco::SingletonHolder<NcAgentNodeManager> sh;
}

NcAgentNodeManager& NcAgentNodeManager::get()
{
    return *sh.get();
}

NcAgentNodeManager::NcAgentNodeManager()
{
    /// default nodes;
    // std::string nodes =
    //     "{\"nodes\":{\"answer_core_bridge\":{\"description\":\"answer 컨트롤
    //     노드\",\"can_control\":false},\"answer_node\":{\"description\":\"slam & localization 통합
    //     노드\",\"can_control\":true},\"base_link_to_laser_link\":{\"description\":\"로봇중심과라이다중심의transform퍼블리시노드\",\"can_control\":false},\"laser_link_to_front_laser\":{\"description\":\"라이다중심과앞라이다중심의transform퍼블리시노드\",\"can_control\":false},\"laser_link_to_rear_laser\":{\"description\":\"라이다중심과뒤라이다중심의transform퍼블리시노드\",\"can_control\":false},\"launch_manager\":{\"description\":\"런치관리노드\",\"can_control\":false},\"nc_brain_map_server\":{\"description\":\"브레인UI를사용하는맵서버노드\",\"can_control\":false},\"nc_calibrator\":{\"description\":\"로봇의바퀴정보를캘리브레이션하는노드\",\"can_control\":false},\"nc_lidar_calibration\":{\"description\":\"라이다transform정보를캘리브레이션하는노드\",\"can_control\":false},\"nc_lidar_merger\":{\"description\":\"여러개의라이다정보를하나의정보로합치는노드\",\"can_control\":false},\"nc_localizer\":{\"description\":\"로봇의위치를추정하는노드\",\"can_control\":false},\"nc_navigator\":{\"description\":\"로봇의네비게이션노드\",\"can_control\":true},\"slam_node\":{\"description\":\"로봇의지도작성을위한노드\",\"can_control\":true},\"robot_state_pub\":{\"description\":\"로봇상태를퍼블리시하는노드\",\"can_control\":false},\"virtual_lidar_node_front\":{\"description\":\"가상의앞라이다정보퍼블리시노드\",\"can_control\":false},\"virtual_lidar_node_rear\":{\"description\":\"가상의뒤라이다정보퍼블리시노드\",\"can_control\":false}}}";

    std::string nodes =
        "{\"nodes\":{\"answer_core_bridge\":{\"description\":\"answer 컨트롤 노드\",\"can_control\":false},\"answer_node\":{\"description\":\"slam & localization 통합 노드\",\"can_control\":false},\"base_link_to_laser_link\":{\"description\":\"로봇중심과라이다중심의transform퍼블리시노드\",\"can_control\":false},\"laser_link_to_front_laser\":{\"description\":\"라이다중심과앞라이다중심의transform퍼블리시노드\",\"can_control\":false},\"laser_link_to_rear_laser\":{\"description\":\"라이다중심과뒤라이다중심의transform퍼블리시노드\",\"can_control\":false},\"launch_manager\":{\"description\":\"런치관리노드\",\"can_control\":false},\"nc_brain_map_server\":{\"description\":\"브레인UI를사용하는맵서버노드\",\"can_control\":false},\"nc_calibrator\":{\"description\":\"로봇의바퀴정보를캘리브레이션하는노드\",\"can_control\":false},\"nc_lidar_calibration\":{\"description\":\"라이다transform정보를캘리브레이션하는노드\",\"can_control\":false},\"nc_lidar_merger\":{\"description\":\"여러개의라이다정보를하나의정보로합치는노드\",\"can_control\":false},\"nc_navigator\":{\"description\":\"로봇의네비게이션노드\",\"can_control\":true},\"robot_state_pub\":{\"description\":\"로봇상태를퍼블리시하는노드\",\"can_control\":false},\"virtual_lidar_node_front\":{\"description\":\"가상의앞라이다정보퍼블리시노드\",\"can_control\":false},\"virtual_lidar_node_rear\":{\"description\":\"가상의뒤라이다정보퍼블리시노드\",\"can_control\":false}}}";

    std::string nodeDataPath = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/configs/ros_nodes.json";

    if (Poco::File(nodeDataPath).exists()) {
        nodes_ = loadJSON(nodeDataPath);
    }
    else {
        nodes_ = JSONParse(nodes);
    }
}

NcAgentNodeManager::~NcAgentNodeManager()
{
    nodes_.reset();
}

void NcAgentNodeManager::updateNodes()
{
    std::vector<std::string> rosNodes;
    ros::master::getNodes(rosNodes);
    auto found = std::string::npos;
    for (const std::string& node : nodes_->getNames()) {
        for (auto current_node : rosNodes) {
            found = current_node.find(node);
            if (found != std::string::npos) {
                break;
            }
        }
        // auto it = std::find(rosNodes.begin(), rosNodes.end(), "/" + node);
        // if (it != rosNodes.end())
        if (found != std::string::npos) {
            nodes_->getObject(node)->set("running", true);
        }
        else {
            nodes_->getObject(node)->set("running", false);
        }
    }
}

std::string NcAgentNodeManager::toString()
{
    std::ostringstream ostr;
    nodes_->stringify(ostr);

    return ostr.str();
}

Poco::JSON::Object::Ptr NcAgentNodeManager::getNodeStatus()
{
    std::vector<std::string> rosNodes;
    ros::master::getNodes(rosNodes);

    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    Poco::JSON::Array arr;
    Poco::JSON::Object::Ptr nodes = nodes_->getObject("nodes");
    auto found = std::string::npos;
    for (const std::string& name : nodes->getNames()) {
        Poco::JSON::Object node;
        node.set("ros_node_id", name);
        node.set("description", nodes->getObject(name)->get("description"));
        node.set("can_control", nodes->getObject(name)->get("can_control"));
        for (auto current_node : rosNodes) {
            found = current_node.find(name);
            if (found != std::string::npos) {
                break;
            }
        }
        if (found != std::string::npos) {
            node.set("running", true);
        }
        else {
            node.set("running", false);
        }
        arr.add(node);
    }
    obj->set("list", arr);

    return std::move(obj);
}
