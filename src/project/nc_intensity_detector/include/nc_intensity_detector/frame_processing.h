#ifndef NC_INTENSITY_DETECTOR_FRAME_PROCESSING_H
#define NC_INTENSITY_DETECTOR_FRAME_PROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "nc_intensity_detector/intensity_detector.h"

namespace nc_intensity_detector {

/**
 * @brief Group corners into potential rectangular frames
 * @param corners Detected corners
 * @param distance Estimated distance to frame
 * @param tape_width Physical width of tape (meters)
 * @param tape_height Physical height of tape (meters)
 * @param focal_length_x Camera focal length in x direction
 * @param focal_length_y Camera focal length in y direction
 * @return Vector of corner groups, each representing a potential frame
 */
std::vector<std::vector<Corner>> groupCornersIntoFrames(
    const std::vector<Corner>& corners,
    double distance,
    double tape_width,
    double tape_height,
    double focal_length_x,
    double focal_length_y);

/**
 * @brief Select the frame closest to image center
 * @param frame_groups Vector of corner groups
 * @param intensity_image Image for center calculation
 * @param principal_point_x Principal point offset in x
 * @param principal_point_y Principal point offset in y
 * @return Selected corner group
 */
std::vector<Corner> selectCenterFrame(
    const std::vector<std::vector<Corner>>& frame_groups,
    const cv::Mat& intensity_image,
    double principal_point_x = 0.0,
    double principal_point_y = 0.0);

/**
 * @brief Validate if corners form a valid frame structure
 * @param corners Corner group to validate
 * @param distance Estimated distance
 * @param tape_width Physical width of tape (meters)
 * @param tape_height Physical height of tape (meters)
 * @param focal_length_x Camera focal length in x direction
 * @param focal_length_y Camera focal length in y direction
 * @return True if corners form a valid frame
 */
bool validateCorners(
    const std::vector<Corner>& corners,
    double distance,
    double tape_width,
    double tape_height,
    double focal_length_x,
    double focal_length_y);

/**
 * @brief Reconstruct frame bounding box from corners
 * @param corners Corner group
 * @return Bounding box rectangle
 */
cv::Rect reconstructFrameFromCorners(const std::vector<Corner>& corners);

/**
 * @brief Calculate center point from corners
 * @param corners Corner group
 * @return Center point
 */
cv::Point2f calculateCenterFromCorners(const std::vector<Corner>& corners);

} // namespace nc_intensity_detector

#endif // NC_INTENSITY_DETECTOR_FRAME_PROCESSING_H

