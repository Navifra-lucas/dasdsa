/**
 * @class data base
 * @brief  Node / Edge data base for graph slam.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */

#ifndef NAVIFRA_DATA_BASE_H
#define NAVIFRA_DATA_BASE_H

#include "2d_slam/common/graph_info.h"
#include "2d_slam/common/slam_parameter_container.h"
#include "2d_slam/globalmapper/mapper.h"
#include "2d_slam/localmapper/localmap/localmap2d.h"
#include "cereal/access.h"
#include "cereal/archives/binary.h"
#include "cereal/archives/json.h"
#include "cereal/cereal.h"
#include "cereal/types/vector.h"
#include "common/math.h"
#include "common/pose2d.h"
#include "glog/logging.h"

#include <deque>
#include <fstream>
#include <memory>
#include <vector>

#include "Eigen/Core"
#define DATA_FILE_EXTENTION ".dat"
using namespace std;
namespace ANSWER {
namespace SLAM2D {
struct grid_point {
    float x = 0.;
    float y = 0.;
    template <class Archive>
    void serialize(Archive &ar)
    {
        ar(cereal::make_nvp("x", x));
        ar(cereal::make_nvp("y", y));
    }
};

class DataBase {
private:
    /* data */
    deque<shared_ptr<Node2D>> all_node_;
    deque<shared_ptr<Edge2D>> all_edge_;
    deque<Edge2D> all_loop_constraints_;
    int n_initial_node_index_ = 0;
    const slam_parameter_container *slam_param_;
    int n_current_index_ = 0;
    float f_total_distance_ = 0.;
    string map_path_;
    string map_name_;
    std::mutex mutex_get_all_node_;
    std::mutex mutex_get_all_edge_;
    std::mutex mutex_get_all_loop_constraint_;

public:
    DataBase(/* args */);
    ~DataBase();

    void Initialize(const string map_path, const string map_name);
    deque<shared_ptr<Node2D>> &GetAllNode()
    {
        std::lock_guard<std::mutex> lock(mutex_get_all_node_);
        return all_node_;
    }

    shared_ptr<Node2D> FindNode(const int index)
    {
        for (auto finding : GetAllNode()) {
            if (finding.get()->index == index) {
                // //NLOG(info) << finding.get()->index << " node found!";
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
        for (size_t i = 0; i < DataBase::GetInstance()->GetAllEdge().size();
             i++) {
            if (DataBase::GetInstance()->GetAllEdge()[i].get()->start->index ==
                    index ||
                DataBase::GetInstance()->GetAllEdge()[i].get()->end->index ==
                    index) {
                // NLOG(info) << "delete edge..";
                DataBase::GetInstance()->GetAllEdge().erase(
                    DataBase::GetInstance()->GetAllEdge().begin() + i);
                i--;
            }
        }

        for (size_t i = 0;
             i < DataBase::GetInstance()->GetLoopConstraints().size(); i++) {
            if (DataBase::GetInstance()->GetLoopConstraints()[i].start->index ==
                    index ||
                DataBase::GetInstance()->GetLoopConstraints()[i].end->index ==
                    index) {
                // NLOG(info) << "delete loop edge..";
                DataBase::GetInstance()->GetLoopConstraints().erase(
                    DataBase::GetInstance()->GetLoopConstraints().begin() + i);
                i--;
            }
        }
    }
    deque<shared_ptr<Edge2D>> &GetAllEdge()
    {
        std::lock_guard<std::mutex> lock(mutex_get_all_edge_);
        return all_edge_;
    }
    deque<Edge2D> &GetLoopConstraints()
    {
        std::lock_guard<std::mutex> lock(mutex_get_all_loop_constraint_);
        return all_loop_constraints_;
    }
    void SetParam(const slam_parameter_container *slam_param)
    {
        slam_param_ = slam_param;
    }
    // const int GetLastNodeIndex() const
    // {
    //     return n_last_node_index_;
    // }
    void AddNode(shared_ptr<Localmap2D> localmap);
    void AddEdge(Node2D *start, Node2D *end);
    void GenerateEdge();
    void GenerateAllEdge();
    static DataBase *GetInstance()
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
    void PublishGlobalMap(const std::vector<int16_t> &global_map);

    void PublishPoseGraph();
};

}  // namespace SLAM2D
}  // namespace ANSWER

#endif