
/*
 * @file	: pose.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Pose (x,y,yaw,robot_angle,s,curv,vel)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_POSE_HPP_
#define NAVIFRA_POSE_HPP_

#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <cmath>
#include <array>
#include <vector>

namespace NVFR {

typedef typename std::array<double, 3UL> Arr3d;

class Pos
{
public:
  using Ptr = std::shared_ptr<Pos>;
  using ConstPtr = std::shared_ptr<const Pos>;

  Pos():x_m(0.0),y_m(0.0),z_m(0.0) {};
  Pos(double _x_m, double _y_m, double _z_m=0.0):x_m(_x_m),y_m(_y_m),z_m(_z_m) {};
  virtual ~Pos() {};

  double GetXm() const { return x_m; };
  double GetYm() const { return y_m; };
  double GetZm() const { return z_m; };

  void SetXm(double val) { x_m=val; };
  void SetYm(double val) { y_m=val; };
  void SetZm(double val) { z_m=val; };
  void SetPos(double _x_m, double _y_m, double _z_m=0.0) { x_m=_x_m; y_m=_y_m; z_m=_z_m; };

  virtual Pos operator+(const Pos& rhs) const;
  virtual Pos operator-(const Pos& rhs) const;

  friend Pos operator*(const Pos& pos, double val);
  friend Pos operator*(double val, const Pos& pos);

  virtual bool operator==(const Pos& rhs) const;
  virtual bool operator!=(const Pos& rhs) const;

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const Pos& o);

protected:
  double x_m;
  double y_m;
  double z_m;

};

inline std::vector<double> GetVecXm(const std::vector<Pos>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetXm();
  }
  return vec;
}

inline std::vector<double> GetVecYm(const std::vector<Pos>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetYm();
  }
  return vec;
}

inline std::vector<double> GetVecZm(const std::vector<Pos>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetZm();
  }
  return vec;
}

class Pose : public Pos
{
public:
  using Ptr = std::shared_ptr<Pose>;
  using ConstPtr = std::shared_ptr<const Pose>;

  // Pose():x_m(0.0),y_m(0.0),yaw(0.0) {};
  Pose():Pos(),yaw(0.0) {};
  Pose(const Pos& o_pos):Pos(o_pos),yaw(0.0) {};
  Pose(double _x_m, double _y_m, double _yaw=0.0);
  Pose(double _x_m, double _y_m, double _z_m, double _yaw);
  Pose(const Arr3d& arr);
  Pose(const std::vector<double>& vec);
  Pose(const Arr3d& p_arr, const Arr3d& v_arr);
  virtual ~Pose() {};

  // double GetXm() const { return x_m; };
  // double GetYm() const { return y_m; };
  double GetRad() const { return yaw; };
  double GetDeg() const { return yaw * 57.29578; };
  double GetRobotRad() const { return robot_yaw; };
  double GetRobotDeg() const { return robot_yaw * 57.29578; };
  double GetSm() const { return s_m; };
  double GetCurv() const { return curv; };
  double GetVX() const { return v_x; };
  double GetVY() const { return v_y; };
  double GetVW() const { return v_w; };
  double GetVL() const { return std::sqrt(v_x*v_x + v_y*v_y); };

  Pos GetPos() const { return Pos(x_m, y_m, z_m); };
  Arr3d GetArr3() const;
  std::vector<double> GetVec3() const;

  // void SetXm(double val) { x_m=val; };
  // void SetYm(double val) { y_m=val; };
  void SetRad(double val);
  void SetDeg(double val);
  void SetPose(double _x_m, double _y_m, double _yaw=0.0);
  void SetPose(const Arr3d& arr);
  void SetPose(const std::vector<double>& vec);
  void SetRobotRad(double val);
  void SetRobotDeg(double val);
  void SetSm(double val) { s_m=val; };
  void SetCurv(double val) { curv=val; };
  void SetVX(double val) { v_x=val; };
  void SetVY(double val) { v_y=val; };
  void SetVW(double val) { v_w=val; };
  void SetVel(double _v_x, double _v_y, double _v_w);
  void SetVel(const Arr3d& arr);

  /**
   * @brief calculate curvature (k = dyaw/ds)
   * @param next_waypoint (next waypoint)
   * @return curvature (k) [1/m]
   */
  void SetCurv(const Pose& next_waypoint);

  /**
   * @brief calculate scalar length (s1 = s0 + ds)
   * @param prev_waypoint (previous waypoint)
   * @return scalar length of this waypoint
   */
  void NextSm(const Pose& prev_waypoint);

  /**
   * @brief calculate scalar length (s1 = s0 + ds)
   * @param prev_waypoint (&) (previous waypoint)
   * @return scalar length of this waypoint
   */
  void NextSm(const Pose* prev_waypoint);

  /**
   * @brief calculate scalar length (s0 = s1 - ds)
   * @param next_waypoint (next waypoint)
   * @return scalar length of this waypoint
   */
  void PrevSm(const Pose& next_waypoint);

  /**
   * @brief calculate scalar length (s0 = s1 - ds)
   * @param next_waypoint (&) (next waypoint)
   * @return scalar length of this waypoint
   */
  void PrevSm(const Pose* next_waypoint);

  /**
   * @brief set speed 0
   */
  void Stop();

  /**
   * @brief check speed 0
   */
  bool IsStop();

  /**
   * @brief reverse robot yaw
   * @return Pose reversed yaw
  */
  Pose ReverseYaw() const;

  /**
   * @brief reverse robot linear velocity
   * @return Pose reversed linear velocity
  */
  Pose ReverseVel() const;

  /**
   * @brief reverse robot states
   * @return Pose reversed states
  */
  Pose Reverse() const;

  /**
   * @brief reverse robot yaw
   * @return void
  */
  void SelfReverseYaw();

  /**
   * @brief reverse robot linear velocity
   * @return void
  */
  void SelfReverseVel();

  /**
   * @brief reverse robot states
   * @return void
  */
  void SelfReverse();

  /**
   * @brief inverse pose
   * @return inversed pose [Pose]
  */
  Pose Inv() const;

  /**
   * @brief calculate next stamp (T + dt) robot pos
   * @param o_robot_state robot pos and vel at time (T)
   * @param dt (time gap)
   * @return robot pos at time (T + dt)
  */
  Pose CalcNextStamp(double dt) const;

  virtual Pose operator+(const Pose& rhs) const;
  virtual Pose operator-(const Pose& rhs) const;
  virtual Pose operator*(const Pose& rhs) const;
  virtual Pose operator/(const Pose& rhs) const;

  friend Pose operator*(const Pose& pose, double val);
  friend Pose operator*(double val, const Pose& pose);

  virtual bool operator==(const Pose& rhs) const;
  virtual bool operator!=(const Pose& rhs) const;

  virtual std::string toStr() const override;
  friend std::ostream& operator<<(std::ostream& os, const Pose& o);

private:
  // double x_m; // pos x [m]
  // double y_m; // pos y [m]
  double yaw; // pos theta [rad]
  double robot_yaw = 0.0; // robot yaw [rad] for QD Windmill Control
  double s_m = 0.0; // scalar length [m]
  double curv = 0.0; // curvature [rad]
  double v_x = 0.0; // linear x velocity [m/s]
  double v_y = 0.0; // linear y velocity [m/s]
  double v_w = 0.0; // angular velocity [rad/s]
};

inline std::vector<double> GetVecXm(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetXm();
  }
  return vec;
}

inline std::vector<double> GetVecYm(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetYm();
  }
  return vec;
}

inline std::vector<double> GetVecSm(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetSm();
  }
  return vec;
}

inline std::vector<double> GetVecVL(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetVL();
  }
  return vec;
}

inline std::vector<double> GetVecVW(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetVW();
  }
  return vec;
}

inline std::vector<double> GetVecCurv(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = v[i].GetCurv();
  }
  return vec;
}

inline std::vector<double> GetVecCurvFromV(const std::vector<Pose>& v)
{
  size_t un_vec_size = v.size();
  std::vector<double> vec(un_vec_size, 0.0);
  for (size_t i=0; i < un_vec_size; ++i) {
    vec[i] = (v[i].GetVL() < 1e-5) ? 0.0 : v[i].GetVW() / v[i].GetVL();
  }
  return vec;
}

typedef typename std::vector< Pos > Polygon;
typedef typename std::vector< Pose > Path;

} // namespace NVFR

#endif
