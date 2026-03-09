#ifndef NAVIFRA_ROBOT_RECT_H
#define NAVIFRA_ROBOT_RECT_H
#include <tf/LinearMath/Vector3.h>
namespace NaviFra {
struct RobotRect {
    tf::Vector3 topLeft;
    tf::Vector3 topRight;
    tf::Vector3 bottomLeft;
    tf::Vector3 bottomRight;
    tf::Vector3 head;  //페드로 요청 사항
};
}  // namespace NaviFra

#endif