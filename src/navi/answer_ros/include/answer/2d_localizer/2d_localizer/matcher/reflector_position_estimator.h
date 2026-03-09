
#pragma once
#include "common/pch.h"
#include "common/pose2d.h"
#include "tsl/robin_map.h"

#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>

namespace ANSWER {

const std::string NOT_FOUND = "not found";
class ReflectorPositionEstimator {
private:
    tsl::robin_map<std::string, Eigen::Vector2d> state_;  // [x, y, vx, vy]
    tsl::robin_map<std::string, Eigen::Vector2d> map_;  // [x, y, vx, vy]
    tsl::robin_map<std::string, std::string> names_;  // [x, y, vx, vy]
    tsl::robin_map<std::string, std::string> names_number_;  // [x, y, vx, vy]
    tsl::robin_map<std::string, Eigen::Matrix2d> P_;  // Covariance
    tsl::robin_map<std::string, Eigen::Matrix2d> F_;  // State transition
    tsl::robin_map<std::string, Eigen::Matrix2d> H_;  // Measurement model
    tsl::robin_map<std::string, Eigen::Matrix2d> R_;  // Measurement noise
    tsl::robin_map<std::string, Eigen::Matrix2d> Q_;  // Process noise
    tsl::robin_map<std::string, unsigned int> touched_;  // touched count
    tsl::robin_map<std::string, bool> published_;  // published status
    tsl::robin_map<std::string, std::chrono::steady_clock::time_point>
        touched_time_;  // touched time

    std::mutex mtx_state_;
    unsigned int map_reflector_count_;

    float f_reflector_distance_m_register_tolerance_;  // meters
    const std::string FindNearestReflector(const Eigen::Vector2d &reflector);
    void Predict(const std::string &uuid);
    bool Update(const std::string &uuid, const Eigen::Vector2d &z);

public:
    ReflectorPositionEstimator(const float f_reflector_distance_tolerance);
    ~ReflectorPositionEstimator();

    void AddMapReflector(
        const std::string &uuid, const std::string &name,
        const Eigen::Vector2d &reflector);
    void UpdateReflector(const Eigen::Vector2d &reflector);
    const tsl::robin_map<std::string, Eigen::Vector2d> GetState()
    {
        std::lock_guard<std::mutex> lock(mtx_state_);
        return state_;
    }
    void RefreshReflectors(const int n_sec);
    const std::string GetUUID(const Eigen::Vector2d &reflector);
    bool empty() const { return state_.empty(); }
    void ClearStates(bool map_set = false)
    {
        if (map_set) {
            std::lock_guard<std::mutex> lock(mtx_state_);
            state_.clear();
            P_.clear();
            F_.clear();
            H_.clear();
            R_.clear();
            Q_.clear();
            touched_.clear();
            touched_time_.clear();
            map_reflector_count_ = 0;
            names_.clear();
            names_number_.clear();
            map_.clear();
            LOG_INFO("cleared all reflector states by set map");
        }
        else {
            for (auto it = state_.begin(); it != state_.end(); ++it) {
                const std::string &uuid = it->first;
                if (map_.find(uuid) != map_.end()) {
                    continue;
                }
                else {
                    std::lock_guard<std::mutex> lock(mtx_state_);
                    state_.erase(uuid);
                    P_.erase(uuid);
                    F_.erase(uuid);
                    H_.erase(uuid);
                    R_.erase(uuid);
                    Q_.erase(uuid);
                    touched_.erase(uuid);
                    touched_time_.erase(uuid);
                    names_.erase(uuid);
                }
            }
            LOG_INFO(
                "cleared all reflector states by init pose --> {}",
                state_.size());
        }
    }
    unsigned int GetTouchedCount(const std::string &uuid)
    {
        std::lock_guard<std::mutex> lock(mtx_state_);
        return touched_[uuid];
    }
    inline const Poco::JSON::Object::Ptr GetAllGlobalReflectorsJSON(
        const int n_reflector_detected_count_condition = 100)
    {
        std::lock_guard<std::mutex> lock(mtx_state_);
        Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
        Poco::JSON::Array::Ptr reflectors = new Poco::JSON::Array();
        for (const auto &[key, position] : state_) {
            if (touched_[key] < n_reflector_detected_count_condition)
                continue;  // Skip if not touched
            Poco::JSON::Object::Ptr point = new Poco::JSON::Object();
            point->set("x", MATH::CutDouble(position.x()));
            point->set("y", MATH::CutDouble(position.y()));
            point->set("uuid", key);
            reflectors->add(point);
        }
        obj->set("reflectors", reflectors);
        return obj;
    }

    const Poco::JSON::Object::Ptr GetNearestLocalReflectorsJSON(
        const Pose2D &map_to_robot, const float &distance_threshold,
        const int n_reflector_detected_count_condition = 100);

    const Poco::JSON::Object::Ptr GetAllLocalReflectorsJSON(
        const Pose2D &robot_to_map,
        const int n_reflector_detected_count_condition = 100);
};

}  // namespace ANSWER