#ifndef GUIK_INITPOSE_HPP
#define GUIK_INITPOSE_HPP

#include "common/pose2d.h"

#include <imgui.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <optional>
#include <sstream>

namespace guik {

class InitPose {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InitPose(
        const std::string &name,
        const Eigen::Matrix4f &init_model_matrix = Eigen::Matrix4f::Identity());

    // void updatePose(const Pose2D &pose);

    void draw_ui();
    void draw_gizmo_ui();
    void draw_gizmo();
    void draw_gizmo(
        int win_x, int win_y, int win_w, int win_h, const Eigen::Matrix4f &view,
        const Eigen::Matrix4f &projection, bool on_window = false);

    bool is_guizmo_using() const;

    const std::string &model_name() const;
    Eigen::Matrix4f model_matrix() const;

    void set_model_matrix(const Eigen::Matrix4f &matrix)
    {
        pose = Eigen::Affine3f(matrix);
    }
    void set_model_matrix(const Eigen::Matrix4d &matrix)
    {
        pose = Eigen::Affine3f(matrix.cast<float>());
    }

    template <typename Scalar, int Mode>
    void set_model_matrix(const Eigen::Transform<Scalar, 3, Mode> &matrix)
    {
        pose = Eigen::Affine3f(matrix.template cast<float>());
    }

    void set_gizmo_enabled(bool enabled);
    void enable_gizmo();
    void disable_gizmo();

    // "TRANSLATE", "ROTATE", "SCALE", "SCALEU", or "UNIVERSAL"
    void set_gizmo_operation(const std::string &operation);
    void set_gizmo_operation(int operation);

    // ImGuizmo mode (LOCAL = 0, WORLD = 1)
    void set_gizmo_mode(int mode);

    // Change the gizmo size (default = 0.1f)
    void set_gizmo_clip_scale(float space = 0.1f);

    bool updatePose(const ANSWER::Pose2D &pose);
    ANSWER::Pose2D getPose();
    bool hasPose();
    void reset();

private:
    std::string name;
    Eigen::Affine3f pose;

    bool gizmo_enabled;
    int gizmo_operation;
    int gizmo_mode;
    float gizmo_clip_space;

    std::mutex pose_mutex_;
    std::optional<ANSWER::Pose2D> cached_;
};

}  // namespace guik

#endif