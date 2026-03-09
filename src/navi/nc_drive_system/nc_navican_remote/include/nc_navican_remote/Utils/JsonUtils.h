#ifndef NAVIFRA_NAVICAN_REMOTE_JSONUTILS
#define NAVIFRA_NAVICAN_REMOTE_JSONUTILS

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <string>

namespace NaviFra {
namespace NaviCAN {
namespace Remote {

// JSON 유틸리티 클래스
class JsonUtils {
public:
    // 성공 응답 생성
    static rapidjson::Document createSuccessResponse(const rapidjson::Document* data = nullptr);

    // 에러 응답 생성
    static rapidjson::Document createErrorResponse(const std::string& error);

    // 문자열을 JSON으로 파싱
    static rapidjson::Document stringToJson(const std::string& str);

    // JSON Document를 문자열로 변환
    static std::string jsonToString(const rapidjson::Document& doc);
    static std::string jsonToString(const rapidjson::Value& val);

    // 안전한 JSON 값 추출 (double)
    static double safeGetDouble(const rapidjson::Value& obj, const char* key, double default_value = 0.0);

    // 안전한 JSON 값 추출 (uint8_t)
    static uint8_t safeGetUint8(const rapidjson::Value& obj, const char* key, uint8_t default_value = 0);

    // 안전한 JSON 값 추출 (string)
    static std::string safeGetString(const rapidjson::Value& obj, const char* key, const std::string& default_value = "");

    // 안전한 JSON 값 추출 (bool)
    static bool safeGetBool(const rapidjson::Value& obj, const char* key, bool default_value = false);

    // 안전한 JSON 값 추출 (int)
    static int safeGetInt(const rapidjson::Value& obj, const char* key, int default_value = 0);

    // JSON 필드 존재 여부 확인
    static bool hasField(const rapidjson::Value& obj, const char* key);
};

}  // namespace Remote
}  // namespace NaviCAN
}  // namespace NaviFra
#endif
