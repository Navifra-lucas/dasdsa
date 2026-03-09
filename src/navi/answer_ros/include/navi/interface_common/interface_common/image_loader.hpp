/*
 * @file	: image_loader.hpp
 * @date	: Mar 27, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parameter manager
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef IMAGE_LOADER_HPP_
#define IMAGE_LOADER_HPP_

#include <string>
#include <tuple>

#include <opencv2/opencv.hpp>

namespace NVFR {

namespace ImageLoader {

bool LoadImageFromYaml(
  std::string directory, const std::string& yaml_file,
  cv::Mat& image, float& offsetX, float& offsetY, float& res);

}  // namespace ImageLoader
namespace IMGL = ImageLoader;
}  // namespace NVFR

#endif
