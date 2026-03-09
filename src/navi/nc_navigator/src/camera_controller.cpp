#include "nc_navigator/nc_navigator_pch.h"

#include <nc_navigator/camera_controller.hpp>

using namespace std;
CameraController::CameraController()
{
    SetParameter();
    RegSubscriber();
    RegService();
}

CameraController::~CameraController()
{
    
}

// 파라미터에 따라 키고 끌 카메라의 인덱스를 설정합니다. 
void CameraController::SetParameter()
{
    bool b_param_set = false;
    nh_.param("camera_controller/front_cameras", vec_vec_camera_index_[CameraDirection::FRONT], {int(0)});
    nh_.param("camera_controller/rear_cameras", vec_vec_camera_index_[CameraDirection::REAR], {int(1)});
    nh_.param("camera_controller/left_cameras", vec_vec_camera_index_[CameraDirection::LEFT], std::vector<int>());
    nh_.param("camera_controller/right_cameras", vec_vec_camera_index_[CameraDirection::RIGHT], std::vector<int>());

    for (const auto& direction : vec_vec_camera_index_) {
        if (direction.size() > 0) b_param_set = true;
    }

    if(!b_param_set) {
        NLOG(error) << "No cameras are set. Please set cameras in the parameter.";
        return;
    }

    NLOG(info) << "Camera parameters set successfully.";     
    
    for(int i = 0; i < vec_vec_camera_index_.size(); i++) {
        if(vec_vec_camera_index_[i].size() > 0) {
            NLOG(info) << "Camera index for direction " << i << ": ";
            for(const auto& index : vec_vec_camera_index_[i]) {
                NLOG(info) << " " << index;
            }
        }
    }
}

// SERVICE 등록
void CameraController::RegSubscriber()
{
    camera_info_sub_ = nh_.subscribe("/nc_obstacle_detector/camera_info", 1, &CameraController::CameraInfoCallback, this);
}

// SERVICE 등록
void CameraController::RegService()
{
    set_camera_client_ = nh_.serviceClient<core_msgs::CameraCmd>("/nc_obstacle_detector/set_camera");
}

void CameraController::CameraInfoCallback(const std_msgs::String::ConstPtr& msg)
{
    if(b_init_state_) return;

    auto parsed_states = ParseCameraStates(msg);
    // 또는 기존 벡터 크기 유지하면서 업데이트
    for (size_t i = 0; i < std::min(parsed_states.size(), vec_change_states_.size()); ++i) {
        vec_change_states_[i] = parsed_states[i];
    }   

    NLOG(info) << "Camera states updated from callback: "
          << "F: " << vec_change_states_[CameraDirection::FRONT]
          << ", R: " << vec_change_states_[CameraDirection::REAR]
          << ", L: " << vec_change_states_[CameraDirection::LEFT]
          << ", R: " << vec_change_states_[CameraDirection::RIGHT];

    b_init_state_ = true; // 초기 상태 설정 완료
}


// 속도 명령 콜백 함수
void CameraController::SelectCamera(const geometry_msgs::Twist msg)
{
    if(!b_init_state_) return;
    // 속도가 0 모두이면 다른 명령없이 현재 상태 유지
    // 이전 명령과 현재 명령이 같다면 현재 상태 유지
    // 속도가 x+면 front 카메라, x-면 rear 카메라 
    // 속도가 y+면 left 카메라, y-면 right 카메라

    vector<bool> vec_change_states = vec_change_states_;

    if(vec_vec_camera_index_[CameraDirection::FRONT].size() == 0 &&
       vec_vec_camera_index_[CameraDirection::REAR].size() == 0 &&
       vec_vec_camera_index_[CameraDirection::LEFT].size() == 0 &&
       vec_vec_camera_index_[CameraDirection::RIGHT].size() == 0) {
        return;
    }
    
    float linear_x = msg.linear.x;
    float linear_y = msg.linear.y;

    b_change_state_ = fabs(linear_x) < 0.01 && fabs(linear_y) < 0.01 ? false : true;

    if(linear_x > 0.01) {
        // front 카메라만 켜기
        vec_change_states[CameraDirection::FRONT] = false;
        vec_change_states[CameraDirection::REAR] = false;
    }
    
    if (linear_x < -0.01) {
        // rear 카메라만 켜기
        vec_change_states[CameraDirection::FRONT] = false;
        vec_change_states[CameraDirection::REAR] = false;
    }

    if (linear_y > 0.01) {
        // left 카메라만 켜기
        vec_change_states[CameraDirection::LEFT] = false;
        vec_change_states[CameraDirection::RIGHT] = false;
    }

    if (linear_y < -0.01) {
        // right 카메라만 켜기
        vec_change_states[CameraDirection::LEFT] = false;
        vec_change_states[CameraDirection::RIGHT] = false;
    }

    if(b_change_state_ && vec_change_states != vec_change_states_) {
        ControllCamera(vec_change_states);
    }
    vec_change_states_ = vec_change_states;
}


void CameraController::ControllCamera(vector<bool> vec_change_states)
{   
    if (vec_vec_camera_index_[CameraDirection::FRONT].size() > 0 ||
        vec_vec_camera_index_[CameraDirection::REAR].size() > 0) {

        if (vec_change_states[CameraDirection::FRONT] != vec_change_states_[CameraDirection::FRONT]) {
            for (const auto& camera_index : vec_vec_camera_index_[CameraDirection::FRONT]) {
                if (CallCameraService(camera_index, vec_change_states[CameraDirection::FRONT]))
                    NLOG(info) << "FRONT camera " << camera_index << " turned changed : "
                               << vec_change_states_[CameraDirection::FRONT] << " -> "
                               << vec_change_states[CameraDirection::FRONT] << ".";
                else
                    NLOG(error) << "Failed to change state FRONT camera " << camera_index;
            }
        }

        if (vec_change_states[CameraDirection::REAR] != vec_change_states_[CameraDirection::REAR]){
            for (const auto& camera_index : vec_vec_camera_index_[CameraDirection::REAR]) {
                if(CallCameraService(camera_index, vec_change_states[CameraDirection::REAR]))
                    NLOG(info) << "REAR camera " << camera_index << " turned changed : "
                               << vec_change_states_[CameraDirection::REAR] << " -> "
                               << vec_change_states[CameraDirection::REAR] << ".";
                else
                    NLOG(error) << "Failed to change state REAR camera " << camera_index;
            }
        }
    }
    
    if (vec_vec_camera_index_[CameraDirection::LEFT].size() > 0 ||
        vec_vec_camera_index_[CameraDirection::RIGHT].size() > 0) {

        if(vec_change_states[CameraDirection::LEFT] != vec_change_states_[CameraDirection::LEFT]) {
            for (const auto& camera_index : vec_vec_camera_index_[CameraDirection::LEFT]) {
                if(CallCameraService(camera_index, vec_change_states[CameraDirection::LEFT]))
                    NLOG(info) << "LEFT camera " << camera_index << " turned changed : "
                               << vec_change_states_[CameraDirection::LEFT] << " -> "
                               << vec_change_states[CameraDirection::LEFT] << ".";
                else
                    NLOG(error) << "Failed to change state LEFT camera " << camera_index;
            }
        }

        if(vec_change_states[CameraDirection::RIGHT] != vec_change_states_[CameraDirection::RIGHT]) {
            for (const auto& camera_index : vec_vec_camera_index_[CameraDirection::RIGHT]) {
                if(CallCameraService(camera_index, vec_change_states[CameraDirection::RIGHT]))
                    NLOG(info) << "RIGHT camera " << camera_index << " turned changed : "
                               << vec_change_states_[CameraDirection::RIGHT] << " -> "
                               << vec_change_states[CameraDirection::RIGHT] << ".";
                else
                    NLOG(error) << "Failed change state RIGHT camera " << camera_index;
            }
        }
    }
}


bool CameraController::CallCameraService(int camera_index, bool use_camera, double timeout_sec) {
    core_msgs::CameraCmd srv;
    srv.request.b_use = use_camera;
    srv.request.index = camera_index;
    
    auto future = std::async(std::launch::async, [this, srv]() mutable {
        return set_camera_client_.call(srv);
    });
    
    if (future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec))) 
        == std::future_status::timeout) {
        LOG_WARNING("Camera service call timed out for camera %d", camera_index);
        return false;
    }
    
    return future.get();
}


std::vector<bool> CameraController::ParseCameraStates(const std_msgs::String::ConstPtr& msg) {
    std::vector<bool> results;
    std::string input = msg->data;
    
    std::regex pattern(R"(:\s*(TRUE|FALSE))");
    std::sregex_iterator iter(input.begin(), input.end(), pattern);
    std::sregex_iterator end;
    
    for (; iter != end; ++iter) {
        std::string value = (*iter)[1].str();
        results.push_back(value == "TRUE");
    }
    
    return results;
}
