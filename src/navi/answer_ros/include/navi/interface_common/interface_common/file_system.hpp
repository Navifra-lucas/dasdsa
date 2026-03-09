/*
 * @file	: file_system.hpp
 * @date	: Mar 27, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parameter manager
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef FILE_SYSTEM_HPP_
#define FILE_SYSTEM_HPP_

#include <string>
#include <filesystem>

namespace NVFR {

namespace FileSystem {

std::string GetHomeDirectory();

bool CreateDirectory(const std::string& directory);

}  // namespace FileSystem
namespace FS = FileSystem;
}  // namespace NVFR

#endif
