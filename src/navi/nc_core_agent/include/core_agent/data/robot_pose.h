#ifndef NAVIFRA_ROBOT_POSE_H
#define NAVIFRA_ROBOT_POSE_H

#include <Poco/JSON/Object.h>
#include <core_agent/data/types.h>

#include <memory>
#include <mutex>

namespace NaviFra {
class RobotPose {
public:
    const static std::string KEY;
    RobotPose() = default;
    ~RobotPose() = default;

    using Ptr = std::shared_ptr<RobotPose>;

    void update(const Position& position, const Orientation& orientation);
    const Poco::JSON::Object::Ptr toObject();

    Position getPosition() const { return position_; }
    Orientation getOrientation() const { return orientation_; }

private:
    Position position_;
    Orientation orientation_;

    std::mutex mutex_;
};

}  // namespace NaviFra
#endif
