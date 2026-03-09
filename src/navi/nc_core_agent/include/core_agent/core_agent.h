#ifndef NAVIFRA_CORE_AGENT_H
#define NAVIFRA_CORE_AGENT_H

#include <Poco/Format.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/UUIDGenerator.h>
#include <core/util/logger.hpp>
#include <core_agent/data/lidar_merger.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/motor_info.h>
#include <core_agent/data/robot_calibration.h>
#include <core_agent/data/robot_collision_info.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_pose.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/data/robot_wheel_info.h>
#include <core_agent/data/types.h>
#include <core_agent/manager/message_handler_manager.h>
#include <core_agent/util/config.h>
#include <core_msgs/CheonilReadRegister.h>
#include <core_msgs/CheonilReadCoil.h>
#include <core_msgs/CommonString.h>
#include <core_msgs/Goal.h>
#include <core_msgs/HacsNode.h>
#include <core_msgs/HacsNodeList.h>
#include <core_msgs/RepeatTestMsg.h>
#include <core_msgs/Sound.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <memory>

#endif  // NAVIFRA_CORE_AGENT_H