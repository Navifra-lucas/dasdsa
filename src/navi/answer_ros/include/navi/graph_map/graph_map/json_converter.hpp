/*
 * @file	: json_converter.hpp
 * @date	: Nov 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: json converter for graph map
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GRAPH_MAP_JSON_CONVERTER_HPP_
#define GRAPH_MAP_JSON_CONVERTER_HPP_

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>

#include <vector>
#include <string>

#include "graph_map/graph_node.hpp"

namespace NVFR {

namespace JsonCVT {

bool convert(
  const Poco::JSON::Object::Ptr&,
  Pose&);

bool convert(
  const Poco::JSON::Object::Ptr&,
  const Poco::JSON::Object::Ptr&,
  Pose&);

bool convert(
  const Poco::JSON::Object::Ptr&,
  GraphNode&);

bool convert(
  const Poco::JSON::Object::Ptr&,
  std::vector<GraphNode>&, std::vector<SimplePath>&);

} // namespace JsonCVT

} // namespace NVFR

#endif
