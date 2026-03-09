#include "ros/service_server.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <filesystem>

#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edit_param");

    std::string file_name;
    if (argc < 2) {
        std::cout << "Usage: rosrun answer edit_param [FILE_NAME]" << std::endl;
        return 0;
    }
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: rosrun answer edit_param [FILE_NAME]"
                      << std::endl;
            std::cout << "File name list: " << std::endl;
            std::cout << "  ros" << std::endl;
            std::cout << "  localizer2d" << std::endl;
            std::cout << "  slam2d" << std::endl;
            return 0;
        }
        else {
            file_name = arg;
        }
    }
    std::string path =
        ros::package::getPath("answer") + "/config/" + file_name + ".json";
    if (!std::filesystem::exists(path)) {
        std::cout << "File does not exist: " << path << std::endl;
        std::cout << "File name list: " << std::endl;
        std::cout << "  ros" << std::endl;
        std::cout << "  localizer2d" << std::endl;
        std::cout << "  slam2d" << std::endl;
        return 1;
    }
    std::cout << "Edit parameter file: " << path << std::endl;
    std::string command = "vi " + path;
    bool run_vi = true;
    bool run_nano = true;

    int result = system(command.c_str());
    if (result == -1) {
        std::cerr << "Error executing vi command: " << command << std::endl;
        run_vi = false;
    }
    if (run_vi == false) {
        command = "nano " + path;
        result = system(command.c_str());
        if (result == -1) {
            std::cerr << "Error executing nano command: " << command
                      << std::endl;
            run_nano = false;
        }
        if (run_nano == false) {
            std::cerr << "No editor available. Exiting." << std::endl;
            return 1;
        }
    }

    std::cout << "\n Do you want to update parameters ? [Y/N] " << path
              << std::endl;
    std::string input;
    std::cin >> input;
    if (input != "Y" && input != "y" && input != "N" && input != "n") {
        std::cout << "Invalid input. Exiting." << std::endl;
        return 1;
    }
    if (input == "N" || input == "n") {
        std::cout << "No update parameters. Exiting." << std::endl;
        return 0;
    }
    result = system("rosrun answer update_param");

    // ros::spin();
    return 0;
}