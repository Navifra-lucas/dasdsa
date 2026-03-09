/*
 * @file	: map.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MAP_HPP
#define NAVIFRA_MAP_HPP

#include "simplepos/simplepos.hpp"
#include "util/util.hpp"

#include <cassert>
#include <iostream>
#include <vector>

namespace NaviFra {

class Map {
public:
  /**
   * @brief Construct a new Map object
   *
   * @param n_size_x_m 지도의 x 크기, 단위는 m
   * @param n_size_y_m 지도의 y 크기, 단위는 m
   * @param f_resolution_m 지도 해상도 기본은 0.05m --> 한 pix이 5cm
   */
  Map(){};
  // Map(0,0 );
  Map(int n_size_x_px, int n_size_y_px, float f_resolution_m = 0.05);
  Map(float f_size_x_m, float f_size_y_m, float f_resolution_m = 0.05);
  Map(float f_size_x_m, float f_size_y_m, float f_origin_x_m,
      float f_origin_y_m, float f_resolution_m = 0.05);
  virtual ~Map(){};
  virtual void
  SetSensorData(const std::vector<NaviFra::SimplePos> &vec_sensors){};

  /**
   * @brief 지도데이터 객체 리턴한다.
   *
   * @return std::vector<int8_t>
   */
  std::vector<int8_t> GetMap() const;
  std::vector<int8_t> GetMap();
  void SetMap(std::vector<int8_t> &vec_map);
  void SetMapInfo(int n_size_x_px, int n_size_y_px,
                  float f_resolution_m = 0.05);
  void SetMapInfo(float f_size_x_m, float f_size_y_m,
                  float f_resolution_m = 0.05);

  const std::vector<int8_t> *GetMapPointer() const;

  std::vector<int8_t> *GetMapPointer();

  /**
   * @brief 지도의 x축 크기(px)를 리턴한다.
   *
   * @return int
   */
  int GetXpx() const;

  /**
   * @brief 지도의 y축 크기(px)를 리턴한다.
   *
   * @return int
   */
  int GetYpx() const;

  /**
   * @brief 지도의 X축 크기(m) 리턴한다.
   *
   * @return float
   */
  float GetXm() const;

  /**
   * @brief //지도의 Y축 크기(m) 리턴한다.
   *
   * @return flaot
   */
  float GetYm() const;

  /**
   * @brief 지도해상도 (m) 리턴
   *
   * @return float
   */
  float getresolutionM() const;
  /**
   * @brief 입력받은 데이터를 지도 class 복제한다
   *
   * @param n_size_x_px 소스지도의 x 크기
   * @param n_size_y_px 소스지도의 y 크기
   * @param pc_data 지도 데이터
   */
  void CloneMap(const int n_size_x_px, const int n_size_y_px,
                const std::vector<signed int8_t> &vec_data);

  float GetXOriginM() const;

  /**
   * @brief 지도의 X축 Origin(m) 리턴한다.
   *
   * @return float
   */

  float GetYOriginM() const;

  void Clear() { vec_map_.clear(); }

  void SetMapOrigin(float f_origin_x_m, float f_origin_y_m);

private:
  int n_size_x_px_, n_size_y_px_; // 지도 크기를 저장하고 있는 변수.
  float f_resolution_m_;          // 격자지도 셀 크기
  float f_size_x_m_, f_size_y_m_; // 미터 단위 지도크기를 저장하고 있는 변수

  float f_origin_x_m_,
      f_origin_y_m_; // 미터 단위 Origin 정보를 저장하고 있는 변수
  int n_origin_x_px_,
      n_origin_y_px_; // 픽셀 단위 Origin 정보를 저장하고 있는 변수

  std::vector<int8_t> vec_map_;
};
} // namespace NaviFra
#endif
