#ifndef SLAM_UTILS_H
#define SLAM_UTILS_H
#include "common/pose2d.h"
#include "common/scan2d.h"

#include <iostream>
#include <sstream>
#include <vector>

// Name Value Pair for SLAM
#define STRINGFY(T) (#T)
namespace NaviFra {
namespace SLAM2D {
namespace slam_utils {

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
 * \brief slam_utils::ColorizeString. Print coloured string
 * in terminal.
 * \param str. Input string.
 * \param colour. Colour option: BLACK, RED,
 * GREEN, YELLOW, BLUE, WHITE.
 * \param option. Char type option: BOLD, REGULAR,
 * UNDERLINE.
 * \return
 */

std::string ColorizeString(std::string str, std::string colour, std::string option = REGULAR);
std::vector<std::string> GetPackagePath();
Scan2D TransformPointCloudToTargetFrame(const Pose2D& a_to_b, const Scan2D& b_to_point);
bool IsEqual(const float& a, const float& b);
bool IsEqual(const double& a, const double& b);

}  // namespace slam_utils
}  // namespace SLAM2D
}  // namespace NaviFra

#endif