/**
 * @class data base
 * @brief  Node / Edge data base for graph slam.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */

#ifndef NAVIFRA_DATA_BASE_H
#define NAVIFRA_DATA_BASE_H

#include "cereal/access.hpp"
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "common/pose2d.h"
#include "common/slam_parameter_container.h"
#include "glog/logging.h"
#include "localmapper/localmap/localmap2d.h"
#include "util/logger.hpp"

#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>

#include <deque>
#include <memory>
#include <vector>

#include "Eigen/Core"
// #include "globalmapper/mapper.h"
#define DATA_FILE_EXTENTION ".dat"
using namespace std;
namespace NaviFra {
namespace SLAM2D {
struct grid_point {
    float x = 0.;
    float y = 0.;
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::make_nvp("x", x));
        ar(cereal::make_nvp("y", y));
    }
};

struct Node2D {
    Node2D()
        : index(0)
        , fixed(false)
    {
    }
    Node2D(int idx, double x, double y, double t, bool fx)
        : index(idx)
        , pose{x, y, t}
        , fixed(fx)
    {
        tag = Poco::UUIDGenerator::defaultGenerator().create();
    }

    ~Node2D() {}
    int index;
    Poco::UUID tag;

    // Pose2D pose;
    double pose[3];  // for pose graph optimization
    bool fixed;
    shared_ptr<Localmap2D> localmap;
    vector<Scan2D> all_raw_scan;
};
struct Edge2D {
    Edge2D() { information = Matrix3d::Zero(); };
    Edge2D(Node2D* s, Node2D* e, Matrix3d inform, bool b_loop = false)
    {
        CHECK_NE(s, nullptr);
        CHECK_NE(e, nullptr);

        start = s;
        end = e;
        CalculateConstraint();
        // constraint = start->pose.inv() * end->pose; // start to end

        information = Matrix3d::Zero();
        information = inform;
        loop_constraint = b_loop;
    };
    ~Edge2D(){};
    Node2D* start;
    Node2D* end;
    double constraint[3];
    Matrix3d information;
    bool loop_constraint = false;

    inline void CalculateConstraint()
    {
        Pose2D start_(Eigen::Vector3f(this->start->pose[0], this->start->pose[1], this->start->pose[2]));
        Pose2D end_(Eigen::Vector3f(this->end->pose[0], this->end->pose[1], this->end->pose[2]));
        auto constraint_ = start_.inv() * end_;  // start to end
        this->constraint[0] = constraint_.GetPose().x();
        this->constraint[1] = constraint_.GetPose().y();
        this->constraint[2] = constraint_.GetPose().z();  // theta
    }
};
class DataBase {
private:
    /* data */
    deque<shared_ptr<Node2D>> all_node_;
    deque<shared_ptr<Edge2D>> all_edge_;
    deque<Edge2D> all_loop_constraints_;
    int n_initial_node_index_ = 0;
    const slam_parameter_container* slam_param_;
    int n_current_index_ = 0;
    float f_total_distance_ = 0.;
    std::ofstream os_;
    string map_path_;
    string map_name_;
    std::mutex mutex_get_all_node_;
    std::mutex mutex_get_all_edge_;
    std::mutex mutex_get_all_loop_constraint_;

public:
    DataBase(/* args */);
    ~DataBase();

    void Initialize(const string map_path, const string map_name);
    deque<shared_ptr<Node2D>>& GetAllNode()
    {
        std::lock_guard<std::mutex> lock(mutex_get_all_node_);
        return all_node_;
    }

    shared_ptr<Node2D> FindNode(const int index)
    {
        for (auto finding : GetAllNode()) {
            if (finding.get()->index == index) {
                // NLOG(info) << finding.get()->index << " node found!";
                return finding;
            }
        }
        CHECK_EQ(1, 1) << index << " node not found";
        return shared_ptr<Node2D>();
    }
    shared_ptr<Node2D> FindNode(const Poco::UUID uuid)
    {
        for (auto finding : GetAllNode()) {
            if (finding.get()->tag == uuid) {
                return finding;
            }
        }
        CHECK_EQ(1, 1) << uuid.toString() << " node not found";
        return shared_ptr<Node2D>();
    }
    void DeleteNode(const int index)
    {
        for (size_t i = 0; i < DataBase::GetInstance()->GetAllEdge().size(); i++) {
            if (DataBase::GetInstance()->GetAllEdge()[i].get()->start->index == index ||
                DataBase::GetInstance()->GetAllEdge()[i].get()->end->index == index) {
                NLOG(info) << "delete edge..";
                DataBase::GetInstance()->GetAllEdge().erase(DataBase::GetInstance()->GetAllEdge().begin() + i);
                i--;
            }
        }

        for (size_t i = 0; i < DataBase::GetInstance()->GetLoopConstraints().size(); i++) {
            if (DataBase::GetInstance()->GetLoopConstraints()[i].start->index == index ||
                DataBase::GetInstance()->GetLoopConstraints()[i].end->index == index) {
                NLOG(info) << "delete loop edge..";
                DataBase::GetInstance()->GetLoopConstraints().erase(DataBase::GetInstance()->GetLoopConstraints().begin() + i);
                i--;
            }
        }
    }
    deque<shared_ptr<Edge2D>>& GetAllEdge()
    {
        std::lock_guard<std::mutex> lock(mutex_get_all_edge_);
        return all_edge_;
    }
    deque<Edge2D>& GetLoopConstraints()
    {
        std::lock_guard<std::mutex> lock(mutex_get_all_loop_constraint_);
        return all_loop_constraints_;
    }
    void SetParam(const slam_parameter_container* slam_param) { slam_param_ = slam_param; }
    // const int GetLastNodeIndex() const
    // {
    //     return n_last_node_index_;
    // }
    void AddNode(shared_ptr<Localmap2D> localmap);
    void AddEdge(Node2D* start, Node2D* end);
    void GenerateEdge();
    void GenerateAllEdge();
    static DataBase* GetInstance()
    {
        static DataBase s;
        return &s;
    }
    void ReadFile(const string map_path, const string map_name);

    void WriteFile(const string map_path, const string map_name);
    void RemoveAddedInformation();
    void SaveHighResolutionMap(std::shared_ptr<Localmap2D> localmap);
    void ReadHighResolutionMap();
    void SavePointCloudJSON(const cv::Mat map_png);
};

}  // namespace SLAM2D
}  // namespace NaviFra

#endif