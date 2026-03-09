#include "core_agent/core_agent.h"

#include <core_agent/data/lidar_merger.h>
#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud.h>

#include <thread>

using namespace NaviFra;

TEST(LidarMergerTest, UpdateAndToString)
{
    LidarMerger merger;

    // PointCloud1
    sensor_msgs::PointCloud::Ptr cloud1(new sensor_msgs::PointCloud);
    cloud1->points.resize(1);
    cloud1->points[0].x = 1.0;
    cloud1->points[0].y = 2.0;
    cloud1->points[0].z = 0.0;

    // PointCloud2
    sensor_msgs::PointCloud::Ptr cloud2(new sensor_msgs::PointCloud);
    cloud2->points.resize(1);
    cloud2->points[0].x = 3.0;
    cloud2->points[0].y = 4.0;
    cloud2->points[0].z = 0.0;

    merger.update(0, cloud1);
    merger.update(1, cloud2);

    std::string expected = "[[[1.000,2.000,0]],[[3.000,4.000,0]]]";
    ASSERT_EQ(merger.toString(), expected);
}

void updatePointCloud(LidarMerger& merger, size_t index, float x, float y, float z)
{
    sensor_msgs::PointCloud::Ptr cloud(new sensor_msgs::PointCloud);
    cloud->points.resize(1);
    cloud->points[0].x = x;
    cloud->points[0].y = y;
    cloud->points[0].z = z;
    merger.update(index, cloud);
}

TEST(LidarMergerTest, ConcurrentUpdate)
{
    LidarMerger merger;

    std::thread t1(updatePointCloud, std::ref(merger), 0, 1.0, 2.0, 0.0);
    std::thread t2(updatePointCloud, std::ref(merger), 1, 3.0, 4.0, 0.0);
    std::thread t3(updatePointCloud, std::ref(merger), 2, 5.0, 6.0, 0.0);
    std::thread t4(updatePointCloud, std::ref(merger), 3, 7.0, 8.0, 0.0);

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    std::string result = merger.toString();
    ASSERT_TRUE(result.find("[1.000,2.000,0]") != std::string::npos);
    ASSERT_TRUE(result.find("[3.000,4.000,0]") != std::string::npos);
    ASSERT_TRUE(result.find("[5.000,6.000,0]") != std::string::npos);
    ASSERT_TRUE(result.find("[7.000,8.000,0]") != std::string::npos);
}
