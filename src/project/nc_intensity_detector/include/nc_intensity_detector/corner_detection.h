#ifndef NC_INTENSITY_DETECTOR_CORNER_DETECTION_H
#define NC_INTENSITY_DETECTOR_CORNER_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "nc_intensity_detector/intensity_detector.h"

namespace nc_intensity_detector {


/**
 * @brief Detect corners using PnP method (contour-based with solvePnP)
 * @param intensity_image Original intensity image (16-bit)
 * @param thresholded Binary thresholded image (8-bit)
 * @param tape_width Physical width of tape in meters
 * @param tape_height Physical height of tape in meters
 * @param focal_length_x Camera focal length in x direction
 * @param focal_length_y Camera focal length in y direction
 * @param distortion_k1 Radial distortion coefficient 1
 * @param distortion_k2 Radial distortion coefficient 2
 * @param distortion_k3 Radial distortion coefficient 3
 * @param distortion_p1 Tangential distortion coefficient 1
 * @param distortion_p2 Tangential distortion coefficient 2
 * @param principal_point_x Principal point x offset (relative to image center)
 * @param principal_point_y Principal point y offset (relative to image center)
 * @param distance Output parameter: estimated distance from solvePnP
 * @param angle Output parameter: estimated yaw angle from solvePnP
 * @param pitch Output parameter: estimated pitch angle from solvePnP
 * @param roll Output parameter: estimated roll angle from solvePnP
 * @param pnp_y Output parameter: Y position from solvePnP tvec[1] (camera coordinate system)
 * @param selection_mode Selection mode: "center" (closest to image center), "left" (leftmost), "right" (rightmost)
 * @param previous_center Previous detection center for temporal consistency (use (-1, -1) if not available)
 * @param previous_distance Previous detection distance for temporal consistency (use -1 if not available)
 * @param previous_y Previous Y position for tracking validation (use 0.0 if not available)
 * @return Vector of detected corners (4 corners in order: TL, TR, BR, BL), empty if failed
 */
std::vector<Corner> detectCornersPnP(const cv::Mat& intensity_image,
                                      const cv::Mat& thresholded,
                                      double tape_width,
                                      double tape_height,
                                      double tape_thickness,
                                      double focal_length_x,
                                      double focal_length_y,
                                      double distortion_k1,
                                      double distortion_k2,
                                      double distortion_k3,
                                      double distortion_p1,
                                      double distortion_p2,
                                      double principal_point_x,
                                      double principal_point_y,
                                      double& distance,
                                      double& angle,
                                      double& pitch,
                                      double& roll,
                                      double& pnp_y,
                                      std::vector<Corner>& inner_corners,
                                      const std::string& selection_mode = "center",
                                      const cv::Point2f& previous_center = cv::Point2f(-1, -1),
                                      double previous_distance = -1.0,
                                      double previous_y = 0.0);


} // namespace nc_intensity_detector

#endif // NC_INTENSITY_DETECTOR_CORNER_DETECTION_H

