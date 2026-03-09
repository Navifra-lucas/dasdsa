

#ifndef NAVIFRA_POLYGON_INFO_HPP_
#define NAVIFRA_POLYGON_INFO_HPP_

#include "core_calculator/core_calculator.hpp"
#include "msg_information/parameter_information/motion_parameter.hpp"
#include "pos/pos.hpp"
#include "simplepos/simplepos.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace NaviFra {

class Polygon {
private:
public:
    std::vector<NaviFra::Pos> o_polygon_vertexs_;
    Polygon() {}
    ~Polygon() {}

    void Clear();
    void AddVertex(const NaviFra::Pos& pos);
    void AddVertexList(const std::vector<float>& vec_polygon);

    float GetRobotWidth();
    float GetRobotHeigth();

    void UpdateVertexSizePlus(const float f_vertex_plus_size);

    void UpdateVertexRotate(const float f_roatate_rad);

    void UpdateVertexSizePlusFLRR(const float f_front_m, const float f_rear_m, const float f_left_m, const float f_right_m);

    void UpdateVertexInversionV();

    void UpdateVertexInversionH();

    void UpdateVertexSizeMax(const NaviFra::Polygon& o_polygon_compare);

    bool GetPointInPolygon(const NaviFra::Pos& o_pos);
};

}  // namespace NaviFra

#endif
