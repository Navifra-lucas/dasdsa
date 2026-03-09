#include "common/3d/scan3d.h"
#include "ros1/answer_ros1.h"
namespace ANSWER {

void AnswerRos1::GetLidar3D(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    Scan3D scan;
    scan.MutablePointCloud().reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
    for (size_t i = 0; i < msg->height * msg->width;
         ++i, ++msg_x, ++msg_y, ++msg_z) {
        scan.MutablePointCloud().emplace_back(*msg_x, *msg_y, *msg_z);
    }

    lidar_stamp_ = msg->header.stamp;

    LocalizeCallback::localization_3d_callback(scan);
}
}  // namespace ANSWER