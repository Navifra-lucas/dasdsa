#include "polygon.hpp"

namespace NaviFra {
void Polygon::Clear()
{
    o_polygon_vertexs_.clear();
}

float Polygon::GetRobotWidth()
{
    float f_robot_radius = 0.1;
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        if (f_robot_radius < fabs(o_polygon_vertexs_[i].GetYm())) {
            f_robot_radius = fabs(o_polygon_vertexs_[i].GetYm());
        }
    }
    return f_robot_radius;
}

float Polygon::GetRobotHeigth()
{
    float f_robot_radius = 0.1;
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        if (f_robot_radius < fabs(o_polygon_vertexs_[i].GetXm())) {
            f_robot_radius = fabs(o_polygon_vertexs_[i].GetXm());
        }
    }
    return f_robot_radius;
}

void Polygon::AddVertex(const NaviFra::Pos& o_pos)
{
    o_polygon_vertexs_.emplace_back(o_pos);
}

void Polygon::AddVertexList(const std::vector<float>& vec_polygon)
{
    AddVertex(NaviFra::Pos(vec_polygon[1], vec_polygon[2]));
    AddVertex(NaviFra::Pos(vec_polygon[0], vec_polygon[2]));
    AddVertex(NaviFra::Pos(vec_polygon[0], vec_polygon[3]));
    AddVertex(NaviFra::Pos(vec_polygon[1], vec_polygon[3]));
}

void Polygon::UpdateVertexSizePlus(const float f_vertex_plus_size)
{
    if (f_vertex_plus_size == 0) {
        return;
    }
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        NaviFra::Pos o_update_pos;
        if (o_polygon_vertexs_[i].GetXm() > 0) {
            o_polygon_vertexs_[i].SetXm(o_polygon_vertexs_[i].GetXm() + f_vertex_plus_size);
        }
        else {
            o_polygon_vertexs_[i].SetXm(o_polygon_vertexs_[i].GetXm() - f_vertex_plus_size);
        }
        if (o_polygon_vertexs_[i].GetYm() > 0) {
            o_polygon_vertexs_[i].SetYm(o_polygon_vertexs_[i].GetYm() + f_vertex_plus_size);
        }
        else {
            o_polygon_vertexs_[i].SetYm(o_polygon_vertexs_[i].GetYm() - f_vertex_plus_size);
        }
    }
}

void Polygon::UpdateVertexSizePlusFLRR(const float f_front_m, const float f_rear_m, const float f_left_m, const float f_right_m)
{
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        NaviFra::Pos o_update_pos;
        if (o_polygon_vertexs_[i].GetXm() > 0) {
            o_polygon_vertexs_[i].SetXm(o_polygon_vertexs_[i].GetXm() + f_front_m);
        }
        else {
            o_polygon_vertexs_[i].SetXm(o_polygon_vertexs_[i].GetXm() - f_rear_m);
        }
        if (o_polygon_vertexs_[i].GetYm() > 0) {
            o_polygon_vertexs_[i].SetYm(o_polygon_vertexs_[i].GetYm() + f_left_m);
        }
        else {
            o_polygon_vertexs_[i].SetYm(o_polygon_vertexs_[i].GetYm() - f_right_m);
        }
    }
}

void Polygon::UpdateVertexRotate(const float f_roatate_rad)
{
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        NaviFra::Pos o_update_pos;
        o_polygon_vertexs_[i] = CoreCalculator::TransformRotationRad_(o_polygon_vertexs_[i], f_roatate_rad);
    }
}

void Polygon::UpdateVertexInversionV()
{
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        NaviFra::Pos o_update_pos;
        o_polygon_vertexs_[i].SetXm(-o_polygon_vertexs_[i].GetXm());
    }
}

void Polygon::UpdateVertexInversionH()
{
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        NaviFra::Pos o_update_pos;
        o_polygon_vertexs_[i].SetYm(-o_polygon_vertexs_[i].GetYm());
    }
}

void Polygon::UpdateVertexSizeMax(const NaviFra::Polygon& o_polygon_compare)
{
    for (int i = 0; i < o_polygon_vertexs_.size(); i++) {
        NaviFra::Pos o_update_pos;
        if (o_polygon_vertexs_[i].GetXm() > 0 && o_polygon_vertexs_[i].GetXm() < o_polygon_compare.o_polygon_vertexs_[i].GetXm())  // front
        {
            o_polygon_vertexs_[i].SetXm(o_polygon_compare.o_polygon_vertexs_[i].GetXm());
        }
        else if (
            o_polygon_vertexs_[i].GetXm() < 0 && o_polygon_vertexs_[i].GetXm() > o_polygon_compare.o_polygon_vertexs_[i].GetXm())  // rear
        {
            o_polygon_vertexs_[i].SetXm(o_polygon_compare.o_polygon_vertexs_[i].GetXm());
        }
        if (o_polygon_vertexs_[i].GetYm() > 0 && o_polygon_vertexs_[i].GetYm() < o_polygon_compare.o_polygon_vertexs_[i].GetYm())  // left
        {
            o_polygon_vertexs_[i].SetYm(o_polygon_compare.o_polygon_vertexs_[i].GetYm());
        }
        else if (
            o_polygon_vertexs_[i].GetYm() < 0 && o_polygon_vertexs_[i].GetYm() > o_polygon_compare.o_polygon_vertexs_[i].GetYm())  // right
        {
            o_polygon_vertexs_[i].SetYm(o_polygon_compare.o_polygon_vertexs_[i].GetYm());
        }
    }
}

bool Polygon::GetPointInPolygon(const NaviFra::Pos& o_pos)
{
    bool b_result = false;

    int n_crosses = 0;  // 교차 횟수

    std::vector<NaviFra::Pos>& vec_poly = o_polygon_vertexs_;
    for (int i = 0; i < vec_poly.size(); i++) {
        int j = (i + 1) % vec_poly.size();

        //점(point)이 선분(vec_poly[i], vec_poly[j])의 y좌표 사이에 있음
        if ((vec_poly[i].GetYm() > o_pos.GetYm()) != (vec_poly[j].GetYm() > o_pos.GetYm())) {
            // atX는 점(o_pos)을 지나는 수평선과 선분(vec_poly[i], vec_poly[j])의 교점
            float f_poly_y = vec_poly[j].GetYm() - vec_poly[i].GetYm();
            if (f_poly_y == 0)
                f_poly_y = 0.00001;
            double d_x =
                (((vec_poly[j].GetXm() - vec_poly[i].GetXm()) / (f_poly_y)) * (o_pos.GetYm() - vec_poly[i].GetYm())) + vec_poly[i].GetXm();

            // d_x가 오른쪽 반직선과의 교점이 맞으면 교점의 개수를 증가시킨다.
            if (o_pos.GetXm() < d_x) {
                n_crosses++;
            }
        }
    }
    // 홀수면 내부, 짝수면 외부에 있음
    if (0 == (n_crosses % 2)) {
        b_result = false;
    }
    else {
        b_result = true;
    }

    return b_result;
}
};  // namespace NaviFra
