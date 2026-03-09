#pragma once
#include "json.hpp"

#include <core_agent/core/navicore.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace NaviFra;

inline std::string getBasePath()
{
    const char* home_dir = getenv("HOME");
    if (!home_dir) {
        throw std::runtime_error("HOME environment variable not set.");
    }
    return std::string(home_dir) + "/navifra_solution/ros/wia_task/";
}

inline std::string getTaskPath()
{
    std::string s_task_path = getBasePath() + "mission.txt";
    return s_task_path;
}

inline std::string getIndexPath()
{
    std::string s_index_path = getBasePath() + "index.txt";
    return s_index_path;
}

inline std::pair<std::string, int> getMission()
{
    const char* home_dir = getenv("HOME");
    if (home_dir == nullptr) {
        std::cerr << "오류: HOME 환경 변수가 설정되지 않았습니다." << std::endl;
        return {"", -1};
    }

    std::string s_mission_path = getTaskPath();
    std::string s_index_path = getIndexPath();

    std::string mission_json_str;
    int start_index = -1;

    // 1. mission.txt 파일 읽기
    std::ifstream mission_file(s_mission_path);
    if (mission_file.is_open()) {
        std::stringstream buffer;
        buffer << mission_file.rdbuf();
        mission_json_str = buffer.str();
        mission_file.close();
    }
    else {
        std::cerr << "오류: mission.txt 파일을 열 수 없습니다." << std::endl;
        return {"", -1};
    }

    // 2. index.txt 파일 읽기
    std::ifstream index_file(s_index_path);
    if (index_file.is_open()) {
        std::string index_str;
        index_file >> index_str;
        try {
            start_index = std::stoi(index_str);
        }
        catch (const std::exception& e) {
            std::cerr << "오류: index.txt의 내용을 정수로 변환할 수 없습니다: " << e.what() << std::endl;
            return {"", -1};
        }
        index_file.close();
    }
    else {
        std::cerr << "오류: index.txt 파일을 열 수 없습니다." << std::endl;
        return {"", -1};
    }

    // 두 파일 모두 성공적으로 읽었으면 pair로 반환
    return {mission_json_str, start_index};
}

inline void saveTaskTxt(const std::string& task_json)
{
    const char* home_dir = getenv("HOME");
    if (home_dir == nullptr) {
        std::cerr << "Error: HOME environment variable not set." << std::endl;
        return;
    }

    std::string s_base_path = getBasePath();
    std::string s_mission_path = getTaskPath();
    std::string s_index_path = getIndexPath();

    mkdir(s_base_path.c_str(), 0775);

    std::ofstream mission_file(s_mission_path);
    if (!mission_file.is_open()) {
        std::cerr << "Error: Cannot open file for writing: " << s_mission_path << std::endl;
        return;
    }

    mission_file << task_json;
    mission_file.close();
}

inline bool checkTaskTxt()
{
    // ... 이전과 동일한 구현 ...
    const char* home_dir = getenv("HOME");
    if (home_dir == nullptr)
        return false;
    std::string base_path = getBasePath();
    std::string s_mission_path = base_path + "task.txt";
    std::string s_index_path = base_path + "index.txt";
    std::ifstream mission_file(s_mission_path);
    if (!mission_file.is_open() || mission_file.peek() == EOF)
        return false;
    std::ifstream index_file(s_index_path);
    if (!index_file.is_open() || index_file.peek() == EOF)
        return false;
    return true;
}

inline void restartTextTask()
{
    NLOG(info) << "미션 재시작을 시도합니다...";

    // 1. 미션 파일(JSON 문자열)과 시작 인덱스 가져오기
    auto mission_data = getMission();
    std::string mission_json_str = mission_data.first;
    int start_index = mission_data.second;

    if (start_index < 0 || mission_json_str.empty()) {
        NLOG(error) << "미션 정보를 가져오는 데 실패했습니다.";
        return;
    }

    try {
        // 2. 원본 mission.txt 내용을 JSON 객체로 파싱
        nlohmann::json original_request_object = nlohmann::json::parse(mission_json_str);

        // 3. 필요한 'data' 필드가 배열 형태로 있는지 유효성 검사
        if (!original_request_object.contains("data") || !original_request_object["data"].is_array()) {
            NLOG(error) << "mission.txt에 'data' 키가 없거나, 해당 필드가 배열이 아닙니다.";
            return;
        }
        const auto& original_tasks = original_request_object["data"];

        // 4. 시작 인덱스가 유효한 범위 내에 있는지 확인
        if (start_index >= original_tasks.size()) {
            NLOG(info) << "시작 인덱스(" << start_index << ")가 미션 개수(" << original_tasks.size()
                       << ")보다 크거나 같아 실행할 미션이 없습니다.";
            return;
        }

        // 5. 'data' 배열을 시작 인덱스부터 새로 구성 (새로 쌓기)
        nlohmann::json new_tasks_array = nlohmann::json::array();
        for (size_t i = start_index; i < original_tasks.size(); ++i) {
            new_tasks_array.push_back(original_tasks[i]);
        }

        // 6. 원본 요청 객체를 복사한 뒤, 'data' 필드를 새로 만든 배열로 교체
        nlohmann::json new_request_object = original_request_object;
        new_request_object["data"] = new_tasks_array;

        // 7. 새로 조립된 JSON 객체를 문자열로 변환하여 서비스 요청 (단 한 번)
        std::string final_request_body = new_request_object.dump(4);

        NLOG(info) << start_index << "번 인덱스부터 재구성된 미션을 서비스로 요청합니다.";
        // bool b_srvCallReq = requestStringSrv("add_task", final_request_body);
        // NLOG(info) << "서비스 요청 결과 : " << (b_srvCallReq ? "성공" : "실패");
    }
    catch (const nlohmann::json::parse_error& e) {
        NLOG(error) << "JSON 파싱 오류: " << e.what();
    }
}

inline void updateIndexText(int idx)
{
    const char* home_dir = getenv("HOME");
    if (!home_dir)
        return;

    std::string base_path = getBasePath();
    mkdir(base_path.c_str(), 0775);

    std::ofstream index_file(base_path + "index.txt");
    if (!index_file.is_open()) {
        std::cerr << "Error: Cannot open index.txt for writing" << std::endl;
        return;
    }

    index_file << idx;
    index_file.close();
}

inline void clearTaskText()
{
    {
        std::ofstream ofs(getTaskPath(), std::ios::trunc);
        if (!ofs.is_open()) {
            std::cerr << "Error: cannot clear " << getTaskPath() << std::endl;
        }
    }

    {
        std::ofstream ofs(getIndexPath(), std::ios::trunc);
        if (!ofs.is_open()) {
            std::cerr << "Error: cannot clear " << getIndexPath() << std::endl;
        }
    }
}