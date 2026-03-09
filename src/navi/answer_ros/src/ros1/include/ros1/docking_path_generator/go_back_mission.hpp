#ifndef GO_BACK_MISSION_HPP_
#define GO_BACK_MISSION_HPP_

#include "virtual_mission.hpp"

namespace NVFR {

class GoBackMission
{
public:
    GoBackMission();
    ~GoBackMission() = default;

    void SetParam(const DPG_Param_t& st_dpg_param);

    void SetNodeF(const Pose& o_pose);
    void SetNodeN(const Pose& o_pose);
    void SetNodeG(const Pose& o_pose);

    /**
     * @brief generate GoBack path (G -> N -> F)
     * @param o_robot_pose
     * @param path generated path (reference output)
     * @return true (normal) -> publish path / false (exception)
     */
    bool GeneratePath(const Pose& o_robot_pose);

private:
    /**
     * @brief check exceptions
     * @param o_fNode F node
     * @param o_nNode N node
     * @param o_gNode G node
     * @param o_robot_pose
     * @return true (exception) / false (normal)
     */
    bool Exceptions(
        const Pose& o_fNode,
        const Pose& o_nNode,
        const Pose& o_gNode,
        const Pose& o_robot_pose,
        const DPG_Param_t& st_dpg_param) const;

    /**
     * @brief check the robot pose (pi) is on the path from point p1 to point p2
     * @param o_p1 start position of the path
     * @param o_p2 end position of the path
     * @param o_pi check position (robot pose)
     * @param d_far_from_path_dist_m allowed distance from the path to the robot (pi)
     * @return true (on the path) / false (far from the path)
     */
    bool IsOnPath(
        const Pose& o_p1,
        const Pose& o_p2,
        const Pose& o_pi,
        const DPG_Param_t& st_dpg_param) const;

    // param
    DataManager<DPG_Param_t> st_dpg_param_;

    // F node (start)
    DataManager<Pose> o_fNode_;

    // N node (middle)
    DataManager<Pose> o_nNode_;

    // G node (goal)
    DataManager<Pose> o_gNode_;

};

}  // namespace NVFR

#endif
