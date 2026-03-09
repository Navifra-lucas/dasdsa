#pragma once

// stl

#include <limits.h>
#include <math.h>
#include <sophus/se2.h>
#include <sophus/se3.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <execution>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

// utils
#include "common/answer_interface.h"
#include "common/answer_status.h"
#include "common/answer_utils.h"
#include "common/callback_slot.h"
#include "common/callbacks.h"
#include "common/configurator.h"
#include "common/exe_arguments.h"
#include "common/keys.h"
#include "common/math.h"
#include "common/msg_converter.h"
#include "common/path.h"
#include "common/wheel_odometry.h"
#include "logger/logger.h"

#include <Poco/Exception.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <filesystem>
//