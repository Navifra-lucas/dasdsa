/*
 * @file	: node_info.hpp
 * @date	: Aug 22, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for node information (id, name, pose)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NODE_INFO_HPP_
#define NODE_INFO_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>

#include "utils/pose.hpp"

namespace NVFR {

/**
 * @namespace NVFR
 * @class NodeInfo
 * @memberof id, name, pose
*/
class NodeInfo
{
public:
  using Ptr = std::shared_ptr<NodeInfo>;
  using ConstPtr = std::shared_ptr<const NodeInfo>;

  NodeInfo() = default;
  NodeInfo(const std::string& s_id, const std::string& s_name, const Pose& o_pose);
  virtual ~NodeInfo() = default;

  const std::string& GetId() const;
  const std::string& GetName() const;
  const Pose& GetConstPose() const;
  Pose GetPose() const;

  void SetNodeInfo(const std::string& s_id, const std::string& s_name, const Pose& o_pose);
  void SetId(const std::string&);
  void SetName(const std::string&);
  void SetPose(const Pose&);

  bool operator==(const NodeInfo& rhs) const;
  bool operator!=(const NodeInfo& rhs) const;

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const NodeInfo& o);

protected:
  std::string s_id_;
  std::string s_name_;
  Pose o_pose_;

private:

};

} // namespace NVFR

#endif
