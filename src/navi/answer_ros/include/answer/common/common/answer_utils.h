#pragma once
#include "common/3d/scan3d.h"
// #include "common/pch.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/types.h"
#include "logger/logger.h"

#include <Poco/Exception.h>
#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

#include <Eigen/Core>

// using Pose3D = Sophus::SE3d;
// Name Value Pair for SLAM
namespace ANSWER {

namespace answer_utils {

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string YELLOW = "3m";
const std::string BLUE = "4m";
const std::string WHITE = "7m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

/*!
 * \brief answer_utils::ColorizeString. Print coloured string
 * in terminal.
 * \param str. Input string.
 * \param colour. Colour option: BLACK, RED,
 * GREEN, YELLOW, BLUE, WHITE.
 * \param option. Char type option: BOLD, REGULAR,
 * UNDERLINE.
 * \return
 */

std::string ColorizeString(
    std::string str, std::string colour, std::string option = REGULAR);
std::vector<std::string> GetPackagePath();
Scan2D TransformPointCloudToTargetFrame(
    const Pose2D &a_to_b, const Scan2D &b_to_point);
bool IsEqual(const float &a, const float &b);
bool IsEqual(const double &a, const double &b);
inline void DumpMemUsage()
{
    // Open /proc/self/statm to read memory usage
    FILE *f = fopen("/proc/self/statm", "rt");
    if (!f)
        return;

    // Read the content of statm
    char str[300];
    size_t n = fread(str, 1, sizeof(str) - 1, f);
    str[n] = '\0';
    fclose(f);

    // Parse the values from statm
    long total_pages = 0, resident_pages = 0, shared_pages = 0;
    if (sscanf(
            str, "%ld %ld %ld", &total_pages, &resident_pages, &shared_pages) !=
        3) {
        LOG_ERROR("Failed to parse /proc/self/statm content");
        return;
    }

    // Get the system page size
    long page_size = sysconf(_SC_PAGESIZE);  // Page size in bytes

    // Convert pages to memory sizes
    long total_memory = total_pages * page_size;  // Total memory in bytes
    long resident_memory =
        resident_pages * page_size;  // Resident memory in bytes
    long shared_memory = shared_pages * page_size;  // Shared memory in bytes

    // Log the memory usage in human-readable format
    LOG_INFO(
        "SLAM MEM USAGE: Total: {} MB, Resident: {} MB, Shared: {} MB\n",
        (total_memory / (1024 * 1024)), (resident_memory / (1024 * 1024)),
        (shared_memory / (1024 * 1024)));

    // Alternatively, print the output to the console
    // printf("SLAM MEM USAGE: Total: %ld KB, Resident: %ld KB, Shared: %ld
    // KB\n",
    //        total_memory / 1024, resident_memory / 1024, shared_memory /
    //        1024);
}
inline const Eigen::Isometry3d ConvertSE3fToIsometry3d(const Pose3D &pose)
{
    Eigen::Matrix3d rotation = pose.so3().matrix().cast<double>();
    Eigen::Vector3d translation = pose.translation().cast<double>();
    Eigen::Isometry3d isometry;
    isometry.translation() = translation;
    isometry.linear() = rotation;
    return isometry;
}

Poco::JSON::Object::Ptr ReadJSONFile(const std::string &file_path);
std::string ConvertJSONToString(const Poco::JSON::Object::Ptr &object);
Poco::JSON::Object::Ptr ConvertStringToJSON(const std::string &json_string);

}  // namespace answer_utils
}  // namespace ANSWER
