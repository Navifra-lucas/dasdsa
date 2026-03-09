#ifndef NC_INTENSITY_DETECTOR_DISTANCE_ESTIMATION_H
#define NC_INTENSITY_DETECTOR_DISTANCE_ESTIMATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "nc_intensity_detector/intensity_detector.h"

namespace nc_intensity_detector {

/**
 * @brief Estimate distance from detected corners
 * @param corners Detected corners
 * @param tape_width Physical width of tape (meters)
 * @param tape_height Physical height of tape (meters)
 * @param focal_length_x Camera focal length in x direction
 * @param focal_length_y Camera focal length in y direction
 * @param min_distance Minimum valid distance (meters)
 * @param max_distance Maximum valid distance (meters)
 * @return Estimated distance in meters
 */
double estimateDistanceFromCorners(
    const std::vector<Corner>& corners,
    double tape_width,
    double tape_height,
    double tape_thickness,
    double focal_length_x,
    double focal_length_y,
    double min_distance,
    double max_distance);

/**
 * @brief Estimate distance from contour bounding box
 * @param bbox Bounding box of contour
 * @param tape_width Physical width of tape (meters)
 * @param tape_height Physical height of tape (meters)
 * @param focal_length_x Camera focal length in x direction
 * @param focal_length_y Camera focal length in y direction
 * @param min_distance Minimum valid distance (meters)
 * @param max_distance Maximum valid distance (meters)
 * @return Estimated distance in meters
 */
double estimateDistanceFromBBox(
    const cv::Rect& bbox,
    double tape_width,
    double tape_height,
    double focal_length_x,
    double focal_length_y,
    double min_distance,
    double max_distance);

} // namespace nc_intensity_detector

#endif // NC_INTENSITY_DETECTOR_DISTANCE_ESTIMATION_H

