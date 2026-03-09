#include "nc_brain_agent/initializer/robot_collision_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_collision_info.h"
#include "core_agent/data/robot_info.h"
#include "util/logger.hpp"

#include <ros/param.h>

namespace NaviFra {

void RobotCollisionInitializer::initialize()
{
    try {
        // ROS Parameter 읽기
        std::vector<float> robot_outline;
        std::vector<float> robot_collision;
        bool use_detection_mode_flag = false;

        ros::param::get("obstacle/outline", robot_outline);
        ros::param::get("obstacle/collision", robot_collision);
        ros::param::get("obstacle/b_use_detection_mode_flag", use_detection_mode_flag);

        float collision_offsetX = 0.f;
        float collision_offsetY = 0.f;

        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        auto robotCollisionInfo = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);

        // 로봇 외곽선 초기화
        if (robot_outline.size() == 4) {
            LOG_INFO("Read Robot Outline successfully.");
            robotCollisionInfo->initShape(
                robot_outline[0], robot_outline[1], robot_outline[3], robot_outline[2], collision_offsetX, collision_offsetY);
        }

        // 로봇 충돌 영역 초기화
        if (robot_collision.size() == 4 && use_detection_mode_flag) {
            LOG_INFO("Read Robot Collision successfully.");
            robotCollisionInfo->initCollision(
                robot_collision[0], robot_collision[1], robot_collision[3], robot_collision[2], collision_offsetX, collision_offsetY);
        }

        // Base 크기 계산
        int n_kinematics = 0;
        float f_base = 0.0f;
        ros::param::get("motion_base/n_kinematics_type", n_kinematics);

        if (n_kinematics == 0) {  // dd
            ros::param::get("driver_wheel/f_FL_wheel_y_m", f_base);
            f_base *= 2;
        }
        else if (n_kinematics == 1) {  // qd
            ros::param::get("driver_wheel/f_FL_wheel_x_m", f_base);
        }
        else if (n_kinematics == 2) {  // sd
            ros::param::get("driver_wheel/f_FL_wheel_x_m", f_base);
        }

        robotInfo->setRobotBase(f_base);

        LOG_INFO("Robot collision & base info initialized successfully.");
    }
    catch (const Poco::Exception& ex) {
        LOG_ERROR("Robot collision initialization error: %s", ex.displayText().c_str());
    }
}

}  // namespace NaviFra
