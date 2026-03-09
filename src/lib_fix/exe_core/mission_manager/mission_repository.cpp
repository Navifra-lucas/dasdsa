#include "mission_repository.hpp"

#include "util/logger.hpp"

namespace NaviFra {
MissionRepository::MissionRepository()
{
    LOG_INFO("Constructor", 1);
}

MissionRepository::~MissionRepository()
{
    LOG_INFO("Desconstructor", 1);
}

void MissionRepository::SetMissionBundle(const std::vector<PathDescription>& vec_path_desc_stack)
{
    LOG_INFO("A new mission has been given!!", 1);

    //미션 다발을 revserse에서 입력한다. 그래야 pop_back()해서 가장 먼저 해야할 미션부터 제거하기 쉬어서..
    vec_path_desc_stack_ = vec_path_desc_stack;
    std::reverse(vec_path_desc_stack_.begin(), vec_path_desc_stack_.end());
    LOG_INFO("Missions to solve::%d", vec_path_desc_stack_.size());
}

PathDescription MissionRepository::GetCurrentMission()
{
    // reverse해서 저장했기 때문에 가장 뒤에 가장 먼저 수행해야 할 미션이 있다
    if (vec_path_desc_stack_.empty() == false) {
        return vec_path_desc_stack_.back();
    }
    else {
        LOG_INFO("No more mission to do!!! empty mission will be provied!!", 1);
        return PathDescription();
    }
}

int MissionRepository::CompleteMission()
{
    LOG_INFO("CompleteMission");
    if (vec_path_desc_stack_.empty() == false) {
        //수행할 미션이 남아 있으면 완료한 미션을 삭제
        //맨 뒤에 있는 것이 가장 최근에 수행 미션 SetMissionBundle 수행 시, reverse수행했기 때문
        vec_path_desc_stack_.pop_back();
    }
    else {
        LOG_INFO("Mission stack is empty!!!", 1);
    }
    return vec_path_desc_stack_.size();
}

int MissionRepository::GetRemainingMissions()
{
    return vec_path_desc_stack_.size();
}

void MissionRepository::ClearMissionStack()
{
    vec_path_desc_stack_.clear();
    LOG_INFO("ClearMissionStack!!! size : %d", vec_path_desc_stack_.size());
}
}  // namespace NaviFra