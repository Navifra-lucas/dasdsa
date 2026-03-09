#include "common/path.h"
#include "ros/package.h"
#include "ros/service_server.h"

#include <Poco/Exception.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <common/configurator.h>
#include <fcntl.h>
#include <logger/logger.h>
#include <ros/ros.h>
#include <sys/file.h>
#include <unistd.h>

#include <filesystem>
#include <string>

using namespace ANSWER;

class record_data {
public:
    record_data()
    {
        auto pkg_path = ros::package::getPath("answer") + "/config";
        Path::GetInstance().SetAllPaths(pkg_path);  // robot_name is not used here
        Configurator::GetInstance().LoadParameters("ros");

        logger_ = std::make_shared<ANSWER::Logger>(
            "record_bag", "info",
            Configurator::GetInstance()
                .GetParamValue("ros", "base", "base_path")
                .convert<std::string>());

        std::vector<std::string> bag_files;
        std::string path = std::string(std::getenv("HOME")) +
            Configurator::GetInstance()
                .GetParamValue("ros", "base", "base_path")
                .convert<std::string>() +
            "/answer/";
        if (!std::filesystem::is_directory(path)) {
            std::filesystem::create_directories(path);
        }
        // filename as date
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        std::string file_name = oss.str() + ".bag";
        bag_files.push_back("rosbag");
        bag_files.push_back("record");
        bag_files.push_back("-O");
        bag_files.push_back(path + file_name);
        // push pack topics
        auto record_topics = Configurator::GetInstance()
                                 .GetParamValue("ros", "topic", "record")
                                 .extract<Poco::JSON::Array::Ptr>();
        for (size_t i = 0; i < record_topics->size(); ++i) {
            auto topic = record_topics->get(i).convert<std::string>();
            bag_files.push_back(topic);
        }

        logger_->RecordBag(bag_files);
    }
    ~record_data() { logger_->TerminateLogger(); }
    ros::NodeHandle nh_;
    ros::ServiceClient answer_service_;

    std::shared_ptr<ANSWER::Logger> logger_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_data");
    record_data record_data;

    ros::spin();
    return 0;
}