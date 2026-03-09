#include "core_msgs/CameraCmd.h"
#include "core_msgs/CameraRoi.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_obstacle.h>

using namespace NaviFra;

NcActionObstacle::NcActionObstacle()
{
    camera_cmd_req = nh.serviceClient<core_msgs::CameraCmd>("/nc_obstacle_detector/set_camera");
    camera_roi_req = nh.serviceClient<core_msgs::CameraRoi>("/nc_obstacle_detector/set_camera_roi");
}

NcActionObstacle::~NcActionObstacle()
{
}

std::string NcActionObstacle::implName()
{
    return "NcActionObstacle";
}

void NcActionObstacle::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        bool obstacleOn = obj->getValue<bool>("data");
        float detectHeight = obj->getValue<float>("camera_detect_height");
        float detectWidth = obj->getValue<float>("camera_detect_width");
        NLOG(info) << detectHeight << " / " << detectWidth;

        NLOG(info) << Poco::format(
            "Received obstacle command: data=%s, height=%.2f, width=%.2f", (obstacleOn ? "true" : "false"), detectHeight, detectWidth);

        core_msgs::CameraCmd srv_cmd;
        core_msgs::CameraRoi srv_roi;

        NLOG(info) << "test";
        // if (camera_cmd_req.waitForExistence(ros::Duration(2.0)) && camera_roi_req.waitForExistence(ros::Duration(2.0))) {
        if (camera_cmd_req.exists() && camera_roi_req.exists()) {
            if (!obstacleOn && (detectHeight != 0.0 && detectWidth != 0.0)) {
                srv_roi.request.n_index = 2;
                srv_roi.request.d_y_th_m = detectWidth;
                srv_roi.request.d_z_th_m = detectHeight;
                srv_roi.request.b_is_docking = false;

                if (camera_roi_req.call(srv_roi)) {
                    NLOG(info) << "ROI updated with height: " << detectHeight << ", width: " << detectWidth;
                }
                else {
                    NLOG(info) << "Failed to call service set_camera_roi.";
                }
            }
            else {
                srv_cmd.request.b_use = true;
                srv_cmd.request.index = -1;
                srv_cmd.request.b_disable = !obstacleOn;  // b_disable이 true면 카메라 끔

                if (camera_cmd_req.call(srv_cmd)) {
                    NLOG(info) << "CAMERA MODE UPDATED" << obstacleOn;

                    if (obstacleOn) {
                        //감지영역 원상복구. param.yaml 그대로 읽어와서 복붙함. 수정 필요
                        srv_roi.request.n_index = 2;
                        srv_roi.request.d_y_th_m = 7.0;
                        srv_roi.request.d_z_th_m = 2.0;
                        srv_roi.request.b_is_docking = false;

                        if (camera_roi_req.call(srv_roi)) {
                            NLOG(info) << "ROI updated with height: " << detectHeight << ", width: " << detectWidth;
                        }
                        else {
                            NLOG(info) << "Failed to call service set_camera_roi.";
                        }
                    }
                }
                else {
                    NLOG(info) << "Failed to call service set_camera.";
                }
            }
        }
        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("message", obstacleOn ? "Obstacle On" : "Obstacle Off");
        response->set("success", false);  // 조건에 따라 true 설정 가능
        response->set("Cmd", "obstacle");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Obstacle Action Exception: " << ex.displayText();
    }
}
