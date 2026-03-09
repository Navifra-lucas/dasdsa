#pragma once

#include "2d_slam/localmapper/localmap/localmap2d.h"
#include "common/pose2d.h"

#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>
namespace ANSWER {
namespace SLAM2D {
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
}  // namespace SLAM2D
}  // namespace ANSWER