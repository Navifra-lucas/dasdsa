/*
 * @file	: map_loader.hpp
 * @date	: Nov 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: map loader for graph map
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GRAPH_MAP_MAP_LOADER_HPP_
#define GRAPH_MAP_MAP_LOADER_HPP_

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>

#include <string>

#include "graph_map/graph_node.hpp"

namespace NVFR {

class MapLoader
{
public:
  MapLoader() = default;
  ~MapLoader() = default;

  const std::string& GetMapUuid() const;
  const std::vector<GraphNode>& GetNodes() const;
  const std::vector<SimplePath>& GetPaths() const;

  bool LoadMap(const std::string& s_file);

private:
  std::string s_map_uuid_;
  std::vector<GraphNode> vec_nodes_;
  std::vector<SimplePath> vec_paths_;

};

} // namespace NVFR

#endif
