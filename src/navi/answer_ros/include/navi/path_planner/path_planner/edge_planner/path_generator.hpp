/*
 * @file	: path_generator.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GENERATOR_HPP_
#define GENERATOR_HPP_

#include <utility>
#include <stdint.h>
#include <algorithm>

#include "utils/pose.hpp"

namespace NVFR {

namespace PathGenerator
{

/**
 * @note type: uint8_t
 * @param SUCCESS, TYPE, INPUT, SHORT, OVER_CURV, SPEED
*/
enum RESULT : uint8_t
{
  SUCCESS = 0,
  TYPE = 1,
  INPUT,
  SHORT,
  OVER_CURV,
  SPEED,
};
static constexpr char* RESULT_NAME[] = {
  (char*)"SUCCESS  ",
  (char*)"TYPE     ",
  (char*)"INPUT    ",
  (char*)"SHORT    ",
  (char*)"OVER_CURV",
  (char*)"SPEED    ",
};

} // namespace Generator

typedef typename std::pair<PathGenerator::RESULT,Path> PathResult;

} // namespace NVFR

#endif
