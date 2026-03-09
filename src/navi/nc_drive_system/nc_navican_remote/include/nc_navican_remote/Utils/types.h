#ifndef NAVIFRA_NAVICAN_REMOTE_TYPES
#define NAVIFRA_NAVICAN_REMOTE_TYPES

#include <boost/asio/ip/udp.hpp>

#include <chrono>
#include <cstdint>
#include <vector>

using udp = boost::asio::ip::udp;

namespace NaviFra {
namespace NaviCAN {
namespace Remote {

// 명령어 타입
enum class CommandType
{
    QUERY,  // 조회 명령 (상태 조회, 데이터 읽기)
    CONTROL,  // 제어 명령 (단일 모터 제어)
    SYSTEM,  // 시스템 명령 (전체 모터 제어, 시스템 상태)
    BROADCAST  // 브로드캐스트 관련 (실시간 데이터 구독)
};

// 모터 실시간 데이터 구조
struct MotorRealtimeData {
    uint8_t motor_id;
    double position;
    double speed;
    double current;
    double voltage;
    int state;
};

// 실시간 데이터 패킷 구조 (WebSocket용)
struct RealtimeDataPacket {
    uint32_t sequence_number;
    uint64_t timestamp;
    std::vector<MotorRealtimeData> motor_data;
};

}  // namespace Remote
}  // namespace NaviCAN
}  // namespace NaviFra

#endif