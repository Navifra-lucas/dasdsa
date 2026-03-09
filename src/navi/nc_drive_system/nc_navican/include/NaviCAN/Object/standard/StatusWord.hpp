#ifndef NAVICAN_NAVICAN_OBJECT_STATUSWORD_HPP
#define NAVICAN_NAVICAN_OBJECT_STATUSWORD_HPP

#include "NaviCAN/Object/BitField.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Standard {
      
class StatusWord : public NaviFra::NaviCAN::Object::BitField<uint16_t>
{
public:
    // 비트 위치 정의
    enum Bits : uint16_t {
        READY_TO_SWITCH_ON = 0,        // 전원 켤 준비 완료
        SWITCHED_ON = 1,               // 전원 켜짐
        OPERATION_ENABLED = 2,         // 동작 활성화됨
        FAULT = 3,                     // 오류 상태
        VOLTAGE_ENABLED = 4,           // 전압 활성화됨 (이 비트는 일부 프로필에서 다른 의미를 가질 수 있음)
        QUICK_STOP = 5,                // 빠른 정지 (0=활성, 1=비활성)
        SWITCH_ON_DISABLED = 6,        // 전원 켜기 비활성화됨
        WARNING = 7,                   // 경고 상태
        MANUFACTURER_SPECIFIC0 = 8,    // 제조사 특정 비트 0
        REMOTE = 9,                    // 원격 제어 (1=원격, 0=로컬)
        TARGET_REACHED = 10,           // 목표 도달 (1=도달, 0=진행 중)
        INTERNAL_LIMIT_ACTIVE = 11,    // 내부 제한 활성화
        OPERATION_MODE_SPECIFIC0 = 12, // 동작 모드별 특정 비트 0
        OPERATION_MODE_SPECIFIC1 = 13, // 동작 모드별 특정 비트 1
        MANUFACTURER_SPECIFIC1 = 14,   // 제조사 특정 비트 1
        MANUFACTURER_SPECIFIC2 = 15    // 제조사 특정 비트 2
    };
    
    StatusWord() = default;
    explicit StatusWord(uint16_t value);

    // CiA402
    bool isNotReadyToSwitchOn() const;
    bool isSwitchOnDisabled() const;
    bool isReadyToSwitchOn() const;
    bool isSwitchedOn() const;
    bool isOperationEnabled() const;
    bool isQuickStopActive() const;
    bool isFaultReactionActive() const;
    bool isFault() const;
    
};

}
}
}
}


#endif