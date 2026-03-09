#ifndef NAVIFRA_ALARM_DATA_H
#define NAVIFRA_ALARM_DATA_H

#include <optional>  // C++17 이상
#include <string>

namespace NaviFra {
class AlarmUtils {
public:
    struct AlarmData {
        std::unordered_map<std::string, int> desc_to_id;
        std::unordered_map<int, std::string> id_to_desc;
    };
    /**
     * @brief 에러 설명(description)으로 에러 ID를 조회합니다.
     * @param description 조회할 에러 설명 문자열
     * @return 에러 ID. 찾지 못하면 std::nullopt를 반환합니다.
     */
    std::optional<int> getAlarmId(const std::string& description);

    /**
     * @brief 에러 ID로 에러 설명(description)을 조회합니다.
     * @param id 조회할 에러 ID
     * @return 에러 설명 문자열. 찾지 못하면 std::nullopt를 반환합니다.
     */
    std::optional<std::string> getAlarmDescription(int id);
};

}  // namespace NaviFra
#endif  // ALARM_UTILS_H