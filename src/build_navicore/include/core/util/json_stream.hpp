#ifndef JSON_STREAM_HPP_
#define JSON_STREAM_HPP_

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <fstream>
#include <cstdio>

using namespace rapidjson;

namespace NaviFra {
class JsonStream {
public:
    explicit JsonStream() = default;
    ~JsonStream() = default;

    bool ReadJsonFromFile(const std::string& file_path)
    {
        std::ifstream file_stream(file_path);
        if (!file_stream.is_open()) {
            return false;
        }

        std::string file_content((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());

        json_file_.Parse(file_content.c_str());

        return true;
    }

    rapidjson::Document& GetJsonFile() { return json_file_; }

private:
    rapidjson::Document json_file_;
};
};  // namespace NaviFra

#endif