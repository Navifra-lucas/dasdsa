#include "nc_wia_agent/util/yaml_util.h"

#include "util/logger.hpp"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>

static std::vector<float> vec_default_margin = {0.0f, 0.0f, 0.0f, 0.0f};

std::vector<float> readYamlParam(const std::string& key, int num)
{
    std::string pkg_path = ros::package::getPath("nc_wia_agent");

    if (pkg_path.empty()) {
        NLOG(error) << "Could not find package 'nc_wia_agent'. Check ROS_PACKAGE_PATH.";
        return vec_default_margin;
    }

    std::string yaml_path = pkg_path + "/config/lccs_field.yaml";

    NLOG(info) << "Key : " << key << " / Num : " << num;
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        std::vector<float> vec_default_margin;
        if (root[key] && root[key]["default"] && root[key]["default"].IsSequence()) {
            for (const auto& v : root[key]["default"]) {
                vec_default_margin.push_back(v.as<float>());
            }
        }
        else {
            NLOG(info) << "[Config] Missing default for " << key << ", fallback to [0,0,0,0]";
            return {0, 0, 0, 0};
        }

        YAML::Node arr = root[key][num];
        if (!arr || !arr.IsSequence()) {
            return vec_default_margin;
        }

        std::vector<float> result;

        for (const auto& v : arr) {
            result.push_back(v.as<float>());
        }

        if (result.size() != vec_default_margin.size()) {
            NLOG(info) << "[Config] Size mismatch for " << key << "." << num << ", using default.";
            return vec_default_margin;
        }

        return result;
    }
    catch (const std::exception& e) {
        NLOG(info) << "[Config] YAML load/parse error: " << e.what() << ", return default";
        return {0, 0, 0, 0};
    }
}