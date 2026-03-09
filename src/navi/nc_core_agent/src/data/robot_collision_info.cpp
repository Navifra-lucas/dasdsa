#include "core_agent/core_agent.h"

#include <core_agent/data/robot_collision_info.h>

using namespace NaviFra;

const std::string RobotCollisionInfo::KEY = "RobotCollisionInfo";

RobotCollisionInfo::RobotCollisionInfo()
{
}

RobotCollisionInfo::RobotCollisionInfo(RobotCollisionInfo& collision)
{
    // 이클래스는 업데이트 할때 실제 사용 하는 값을 새로 고침 하기 때문에 복사 생성자에서 데이터를 복사 해줄 필요가 없음
}

RobotCollisionInfo::~RobotCollisionInfo()
{
}

void RobotCollisionInfo::initShape(float top, float bottom, float left, float right, float offsetX, float offsetY)
{
    shape_.clear();
    float shapeTop = top + offsetX;
    float shapeBottom = bottom + offsetX;
    float shapeLeft = left + offsetY;
    float shapeRight = right + offsetY;

    shapeRect_.topLeft.setX(shapeTop);
    shapeRect_.topLeft.setY(shapeLeft);
    shapeRect_.topLeft.setZ(0);

    shapeRect_.topRight.setX(shapeTop);
    shapeRect_.topRight.setY(shapeRight);
    shapeRect_.topRight.setZ(0);

    shapeRect_.bottomLeft.setX(shapeBottom);
    shapeRect_.bottomLeft.setY(shapeLeft);
    shapeRect_.bottomLeft.setZ(0);

    shapeRect_.bottomRight.setX(shapeBottom);
    shapeRect_.bottomRight.setY(shapeRight);
    shapeRect_.bottomRight.setZ(0);

    shapeRect_.head.setX(shapeTop);
    shapeRect_.head.setY(0);
    shapeRect_.head.setZ(0);

    Poco::JSON::Array arrhead, arrtopright, arrbottomright, arrbottomleft, arrtopleft;

    arrhead.add(shapeRect_.head.getX());
    arrhead.add(shapeRect_.head.getY());
    arrhead.add(shapeRect_.head.getZ());

    arrtopright.add(shapeRect_.topRight.getX());
    arrtopright.add(shapeRect_.topRight.getY());
    arrtopright.add(shapeRect_.topRight.getZ());

    arrbottomright.add(shapeRect_.bottomRight.getX());
    arrbottomright.add(shapeRect_.bottomRight.getY());
    arrbottomright.add(shapeRect_.bottomRight.getZ());

    arrbottomleft.add(shapeRect_.bottomLeft.getX());
    arrbottomleft.add(shapeRect_.bottomLeft.getY());
    arrbottomleft.add(shapeRect_.bottomLeft.getZ());

    arrtopleft.add(shapeRect_.topLeft.getX());
    arrtopleft.add(shapeRect_.topLeft.getY());
    arrtopleft.add(shapeRect_.topLeft.getZ());

    shape_.add(arrhead);
    shape_.add(arrtopright);
    shape_.add(arrbottomright);
    shape_.add(arrbottomleft);
    shape_.add(arrtopleft);
}

void RobotCollisionInfo::initCollision(float top, float bottom, float left, float right, float offsetX, float offsetY)
{
    collision_.clear();

    float shapeTop = top + offsetX;
    float shapeBottom = bottom + offsetX;
    float shapeLeft = left + offsetY;
    float shapeRight = right + offsetY;

    collisionRect_.topLeft.setX(shapeTop);
    collisionRect_.topLeft.setY(shapeLeft);
    collisionRect_.topLeft.setZ(0);

    collisionRect_.topRight.setX(shapeTop);
    collisionRect_.topRight.setY(shapeRight);
    collisionRect_.topRight.setZ(0);

    collisionRect_.bottomLeft.setX(shapeBottom);
    collisionRect_.bottomLeft.setY(shapeLeft);
    collisionRect_.bottomLeft.setZ(0);

    collisionRect_.bottomRight.setX(shapeBottom);
    collisionRect_.bottomRight.setY(shapeRight);
    collisionRect_.bottomRight.setZ(0);

    collisionRect_.head.setX(shapeTop);
    collisionRect_.head.setY(0);
    collisionRect_.head.setZ(0);

    Poco::JSON::Array arrhead, arrtopright, arrbottomright, arrbottomleft, arrtopleft;

    arrhead.add(collisionRect_.head.getX());
    arrhead.add(collisionRect_.head.getY());
    arrhead.add(collisionRect_.head.getZ());

    arrtopright.add(collisionRect_.topRight.getX());
    arrtopright.add(collisionRect_.topRight.getY());
    arrtopright.add(collisionRect_.topRight.getZ());

    arrbottomright.add(collisionRect_.bottomRight.getX());
    arrbottomright.add(collisionRect_.bottomRight.getY());
    arrbottomright.add(collisionRect_.bottomRight.getZ());

    arrbottomleft.add(collisionRect_.bottomLeft.getX());
    arrbottomleft.add(collisionRect_.bottomLeft.getY());
    arrbottomleft.add(collisionRect_.bottomLeft.getZ());

    arrtopleft.add(collisionRect_.topLeft.getX());
    arrtopleft.add(collisionRect_.topLeft.getY());
    arrtopleft.add(collisionRect_.topLeft.getZ());

    collision_.add(arrhead);
    collision_.add(arrtopright);
    collision_.add(arrbottomright);
    collision_.add(arrbottomleft);
    collision_.add(arrtopleft);
}

void RobotCollisionInfo::updateCollision(std::vector<Position> position)
{
    if (position.size() == 4) {
        Poco::JSON::Array arrhead, arrtopright, arrbottomright, arrbottomleft, arrtopleft;

        arrhead.add(position[0].x);
        arrhead.add(0);
        arrhead.add(0);

        arrtopright.add(position[0].x);
        arrtopright.add(position[0].y);
        arrtopright.add(0);

        arrbottomright.add(position[1].x);
        arrbottomright.add(position[1].y);
        arrbottomright.add(0);

        arrbottomleft.add(position[2].x);
        arrbottomleft.add(position[2].y);
        arrbottomleft.add(0);

        arrtopleft.add(position[3].x);
        arrtopleft.add(position[3].y);
        arrtopleft.add(0);

        {
            Poco::FastMutex::ScopedLock lock(fastMutex_);
            collision_.clear();
            collision_.add(arrhead);
            collision_.add(arrtopright);
            collision_.add(arrbottomright);
            collision_.add(arrbottomleft);
            collision_.add(arrtopleft);
        }
    }
}

Poco::JSON::Array RobotCollisionInfo::getCollision()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return collision_;
}

Poco::JSON::Array RobotCollisionInfo::getShape() const
{
    return shape_;
}

std::vector<Position> RobotCollisionInfo::getCollisionVector() const
{
    std::vector<Position> result;

    Poco::FastMutex::ScopedLock lock(fastMutex_);

    for (size_t i = 0; i < collision_.size(); ++i) {
        auto arr = collision_.getArray(i);
        if (!arr || arr->size() < 3)
            continue;

        Position pos;
        pos.x = arr->getElement<double>(0);
        pos.y = arr->getElement<double>(1);
        pos.z = arr->getElement<double>(2);
        result.push_back(pos);
    }

    return result;
}

std::vector<Position> RobotCollisionInfo::getShapeVector() const
{
    std::vector<Position> result;

    // shape_은 const 함수 안에서도 변경되지 않으므로 별도 mutex 보호 필요 없음
    // (shape_이 initShape()로만 초기화되고 이후 read-only라면)
    // 만약 runtime 중 shape_도 변경 가능하면 FastMutex 사용 필요
    // Poco::FastMutex::ScopedLock lock(fastMutex_);

    for (size_t i = 0; i < shape_.size(); ++i) {
        auto arr = shape_.getArray(i);
        if (!arr || arr->size() < 3)
            continue;

        Position pos;
        pos.x = arr->getElement<double>(0);
        pos.y = arr->getElement<double>(1);
        pos.z = arr->getElement<double>(2);
        result.push_back(pos);
    }

    return result;
}