/*
 * @file	: mision_repository.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	:  로봇이 받은 미션을 저장하고 검사하고 알려주는 기능
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MISSION_REPOSITORY_H_
#define NAVIFRA_MISSION_REPOSITORY_H_

#include "pos/pos.hpp"


#include <algorithm>
#include <iostream>
#include <mutex>
#include <vector>
using namespace std;

namespace NaviFra {
struct PathDescription {
    std::vector<Pos> vec_path;
    Pos o_goal_pos;
    string s_start_name;
    string s_start_node_id;
    string s_goal_name;
    string s_goal_node_id;
    float f_yaw_bias = 0.0f;
    int n_start_align_flag = 0;
    int n_end_align_flag = 0;
};

class MissionRepository {
public:
    MissionRepository();
    virtual ~MissionRepository();

    /**
     * @brief 미션 다발을 설정
     *
     * @param vec_path_desc_stack
     */
    void SetMissionBundle(const std::vector<PathDescription>& vec_path_desc_stack);

    /**
     * @brief 수행해야할 미션을 제공
     *
     * @return PathDescription
     */
    PathDescription GetCurrentMission();

    /**
     * @brief 외부에서 호출하게 되면 미션 stack에서 가장 최근에 수행된 미션이 삭제된다.
     *
     */
    int CompleteMission();

    /**
     * @brief 현재 리포에 남아있는 미션 수 제공
     *
     * @return int
     */
    int GetRemainingMissions();

    /**
     * @brief 미션 스텍에 있는 모든 미션을 지움
     *
     */
    void ClearMissionStack();

    void SetTermiateState(bool flag) { b_terminal_flag_ = flag; }

    bool GetTermiateState() { return b_terminal_flag_; }

private:
    std::vector<PathDescription> vec_path_desc_stack_;
    bool b_terminal_flag_ = false;
};
}  // namespace NaviFra

#endif