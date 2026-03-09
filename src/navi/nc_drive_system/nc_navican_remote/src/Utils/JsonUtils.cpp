#include "nc_navican_remote/Utils/JsonUtils.h"

#include <rapidjson/error/en.h>

using namespace NaviFra::NaviCAN::Remote;
using namespace rapidjson;

Document JsonUtils::createSuccessResponse(const Document* data)
{
    Document response;
    response.SetObject();
    auto& allocator = response.GetAllocator();

    response.AddMember("success", true, allocator);

    if (data != nullptr && data->IsObject()) {
        for (auto it = data->MemberBegin(); it != data->MemberEnd(); ++it) {
            Value key(it->name, allocator);
            Value value(it->value, allocator);
            response.AddMember(key, value, allocator);
        }
    }

    return response;
}

Document JsonUtils::createErrorResponse(const std::string& error)
{
    Document response;
    response.SetObject();
    auto& allocator = response.GetAllocator();

    response.AddMember("success", false, allocator);
    Value errorValue(error.c_str(), error.length(), allocator);
    response.AddMember("error", errorValue, allocator);

    return response;
}

Document JsonUtils::stringToJson(const std::string& str)
{
    Document doc;
    ParseResult result = doc.Parse(str.c_str());

    if (!result) {
        throw std::runtime_error(std::string("JSON parse error: ") + GetParseError_En(result.Code()) + " at offset " +
                                 std::to_string(result.Offset()));
    }

    return doc;
}

std::string JsonUtils::jsonToString(const Document& doc)
{
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
}

std::string JsonUtils::jsonToString(const Value& val)
{
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    val.Accept(writer);
    return buffer.GetString();
}

double JsonUtils::safeGetDouble(const Value& obj, const char* key, double default_value)
{
    if (obj.HasMember(key) && obj[key].IsNumber()) {
        return obj[key].GetDouble();
    }
    return default_value;
}

uint8_t JsonUtils::safeGetUint8(const Value& obj, const char* key, uint8_t default_value)
{
    if (obj.HasMember(key) && obj[key].IsUint()) {
        return static_cast<uint8_t>(obj[key].GetUint());
    }
    return default_value;
}

std::string JsonUtils::safeGetString(const Value& obj, const char* key, const std::string& default_value)
{
    if (obj.HasMember(key) && obj[key].IsString()) {
        return obj[key].GetString();
    }
    return default_value;
}

bool JsonUtils::safeGetBool(const Value& obj, const char* key, bool default_value)
{
    if (obj.HasMember(key) && obj[key].IsBool()) {
        return obj[key].GetBool();
    }
    return default_value;
}

int JsonUtils::safeGetInt(const Value& obj, const char* key, int default_value)
{
    if (obj.HasMember(key) && obj[key].IsInt()) {
        return obj[key].GetInt();
    }
    return default_value;
}

bool JsonUtils::hasField(const Value& obj, const char* key)
{
    return obj.HasMember(key) && !obj[key].IsNull();
}
