#ifndef NAVIFRA_ROBOT_COLLISION_INFO_H
#define NAVIFRA_ROBOT_COLLISION_INFO_H

#include <Poco/JSON/Array.h>
#include <Poco/Mutex.h>
#include <core_agent/data/types.h>

#include <memory>
#include <vector>

namespace NaviFra {
class RobotCollisionInfo {
public:
    RobotCollisionInfo();
    RobotCollisionInfo(RobotCollisionInfo& collision);
    ~RobotCollisionInfo();

    using Ptr = std::shared_ptr<RobotCollisionInfo>;

    const static std::string KEY;

    void initCollision(float top, float bottom, float left, float right, float offsetX, float offsetY);
    void initShape(float top, float bottom, float left, float right, float offsetX, float offsetY);
    void appendPath(float x, float y, float z);

    void updateCollision(std::vector<Position> position);

    Poco::JSON::Array getCollision();
    Poco::JSON::Array getShape() const;

    std::vector<Position> getCollisionVector() const;
    std::vector<Position> getShapeVector() const;

private:
    RobotRect shapeRect_;
    RobotRect collisionRect_;
    Poco::JSON::Array shape_;
    Poco::JSON::Array collision_;

    mutable Poco::FastMutex fastMutex_;
};
}  // namespace NaviFra

#endif  // NC_AGENT_ROBOT_COLLISION_INFO_H