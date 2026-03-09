#ifndef NC_TASK_MANAGER_PCH_H
#define NC_TASK_MANAGER_PCH_H

#include "core/util/logger.hpp"
#include "nc_task_manager/nc_task_manager_types.h"

#include <Poco/JSON/Object.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <boost/format.hpp>
#include <nc_task_manager/data/nc_task.h>
#include <ros/ros.h>

#include <queue>
#include <string>
#include <vector>

#endif  // NC_TASK_MANAGER_PCH_H