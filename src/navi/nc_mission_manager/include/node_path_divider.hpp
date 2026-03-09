/*
 * @file	: node_path_divider.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	: 노드 패스 플래너로부터 나오는 패스를 각 노드의 설정에 따라 분할해서 stack에 쌓아 제공
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_NODE_PATH_DIVIDER_H_
#define NAVIFRA_NODE_PATH_DIVIDER_H_

#include "core_calculator/core_calculator.hpp"
#include "debug/debug_visualizer.hpp"
#include "mission_manager/mission_repository.hpp"
#include "msg_information/parameter_information/param_msg.hpp"
#include "msg_information/sensor_information/sensor_msg.hpp"
#include "param_repository/param_repository.hpp"
#include "pos/pos.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

namespace NaviFra {

class NodePathDivider {
public:
    NodePathDivider();
    virtual ~NodePathDivider();

    /**
     * @brief 주어진 경로를 노드 및 경로의 특성(spin-turn, etc) 에 따라서 분할해 path stack을 제공
     *
     * @param vec_path 경로
     * @return std::vector<std::vector<NaviFra::Pos>> 노드 및 경로 특성에 따라 분할되어 나누어진 경로 집합
     */
    std::vector<std::vector<NaviFra::Pos>> DevidePath(
        const std::vector<NaviFra::Pos>& vec_path, float f_angle_dividing_the_path_deg = 10.f);

    /**
     * @brief Apply divided path in motion (nate)
     *
     * @param path_stack
     * @param yaw_bias
     * @param o_robot_Pos
     * @param vec_out_path_desc_stack
     */
    void ApplyDividedPath(
        std::vector<std::vector<NaviFra::Pos>>&& path_stack, float yaw_bias, const NaviFra::Pos& o_robot_Pos,
        std::vector<PathDescription>& vec_out_path_desc_stack, Parameters_t& st_parameter);

    /**
     * @brief Make Digonal linear curve path (nate)
     *
     * @param vec_path
     * @return std::vector<NaviFra::Pos>
     */
    std::vector<NaviFra::Pos> MakeDigonalPath(std::vector<NaviFra::Pos>& vec_path, Parameters_t& st_parameter);

    /**
     * @brief diagonal path가 로봇 기구학적으로 못 따라 갈 수 있기 때문에 후처리 (nate)
     *
     * @param vec_path
     * @return std::vector<NaviFra::Pos>
     */
    static std::vector<NaviFra::Pos> PostProcessDigonalPath(const std::vector<NaviFra::Pos>& vec_path);
};
}  // namespace NaviFra

#endif
