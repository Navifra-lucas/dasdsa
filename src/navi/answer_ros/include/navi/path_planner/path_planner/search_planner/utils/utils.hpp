/*
 * @file	: utils.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: util for global planner
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_UTILS_HPP_
#define GLOBAL_PLANNER_UTILS_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <stdint.h>

#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"
#include "utils/grid_map/grid_struct.hpp"

namespace NVFR {

namespace SearchPlannerUtils {

/**
 * @note type: unsigned char
 * @param NONE, VISIT, CLOSE, UNKNOWN, OBS
*/
enum class STATUS : unsigned char {
  NONE = 0,
  VISIT = 1,
  CLOSE,
  UNKNOWN,
  OBS,
};

/**
 * @brief Node for global planner
 * @param x pos x in grid map
 * @param y pos y in grid map
 * @param th theta in grid map
 * @param p_cost cost to reach upto this node (= parent->g_cost + d_cost) (d_cost : distance, dynamic, ect ...)
 * @param h_cost heuristic cost fo this node
 * @param r_cost repulsive cost fo this node
 * @param parent index of parent node
 */
class Node
{
public:
  Node():
    x(0),y(0),th(0),p_cost(.0),h_cost(.0),r_cost(.0) {}
  Node(int16_t _x, int16_t _y, int16_t _th=0):
    x(_x),y(_y),th(_th),p_cost(.0),h_cost(.0),r_cost(.0) {}
  Node(int16_t _x, int16_t _y, float _p_cost, float _h_cost, float _r_cost):
    x(_x),y(_y),th(0),p_cost(_p_cost),h_cost(_h_cost),r_cost(_r_cost) {}
  Node(int16_t _x, int16_t _y, int16_t _th, float _p_cost, float _h_cost, float _r_cost):
    x(_x),y(_y),th(_th),p_cost(_p_cost),h_cost(_h_cost),r_cost(_r_cost) {}
  virtual ~Node() = default;

  int16_t x;
  int16_t y;
  int16_t th;

  float p_cost;
  float h_cost;
  float r_cost;

  STATUS e_status = STATUS::NONE;
  Node* parent = nullptr;

  float CalcCost() const;

  Node operator+(const Node& node) const;
  Node operator-(const Node& node) const;
  bool operator==(const Node& node) const; // compare grid pos
  bool operator!=(const Node& node) const; // compare grid pos
  bool operator>(const Node& node) const; // compare total cost
  bool operator<(const Node& node) const; // compare total cost

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& node);
};

/**
 * @brief Node for hybrid A-star in global planner
 * @param x pos x in grid map
 * @param y pos y in grid map
 * @param th theta in grid map
 * @param x_m pos x in world map
 * @param y_m pos y in world map
 * @param yaw angle in world map
 * @param p_cost cost to reach upto this node (= parent->g_cost + d_cost) (d_cost : distance, dynamic, ect ...)
 * @param h_cost heuristic cost fo this node
 * @param r_cost repulsive cost fo this node
 * @param parent index of parent node
 */
class HNode : public Node
{
public:
  HNode():
    Node() {}
  HNode(int16_t _x, int16_t _y, int16_t _th):
    Node(_x, _y, _th),x_m(.0),y_m(.0) {}
  HNode(int16_t _x, int16_t _y, float _p_cost, float _h_cost, float _r_cost):
    Node(_x, _y, _p_cost, _h_cost, _r_cost),x_m(.0),y_m(.0) {}
  HNode(int16_t _x, int16_t _y, int16_t _th, float _p_cost, float _h_cost, float _r_cost):
    Node(_x, _y, _th, _p_cost, _h_cost, _r_cost),x_m(.0),y_m(.0) {}
  HNode(float _x_m, float _y_m, int16_t _th, float _p_cost):
    Node(.0, .0, _th),x_m(_x_m),y_m(_y_m) { p_cost = _p_cost; }
  HNode(const Node& node):
    Node(node),x_m(.0),y_m(.0) {}
  virtual ~HNode() = default;

  float x_m;
  float y_m;
  float yaw;

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const HNode& node);
};

// Node*
struct CmpCost {
  bool operator() (const Node* node1, const Node* node2) const;
};

// normalize angle
int16_t NormalizeAngle(int16_t th, int16_t theta_size);

} // namespace SearchPlannerUtils
namespace SPU = SearchPlannerUtils;
} // namespace NVFR

#endif
