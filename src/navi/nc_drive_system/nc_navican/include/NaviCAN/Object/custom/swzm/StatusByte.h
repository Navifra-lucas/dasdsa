#ifndef NAVICAN_OBJECT_SWZM_STATUS_BYTE_HPP
#define NAVICAN_OBJECT_SWZM_STATUS_BYTE_HPP

#include "NaviCAN/Object/BitField.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Custom {
namespace swzm {

class StatusByte : public NaviFra::NaviCAN::Object::BitField<uint8_t>
{
public:
    // 비트 위치 정의
    enum Bits : uint8_t {
        POWER_STAGE_ACTIVE = 0,        // 0 : 전원 스테이지 비활성화, 1 : 전원 스테이지 활성화
        FAULT = 1,                     // 0 : 정상, 1 : 오류 발생
        TORQUE_MODE = 2,               // 0 : Motor 모드, 1 : Generator 모드
        MODE_ANALOG = 3,               // 0 : CAN 모드, 1 : Analog 모드 (C1p33)
        ENABLE_INPUT = 4,              // 0 : Not Connected, 1 : Input active(C1p31, Analog Mode : Cw 방향, CAN Mode : CAN Enable)
        CCW_INPUT  = 5,                // 0 : Not Connected, 1 : Input active(C1p32, Analog Mode : CCW 방향, CAN Mode : 의미 없음)
        SAFE_STOP_ACTIVE = 6,          // 0 : Safe Stop 비활성화 상태, 1 : Safe Stop 활성화 상태
        TOGGLE_BIT = 7,                // 0 : Togglebit 리셋, 1 : Togglebit 셋, 매 TPDO1 전송 때마다 토글됌
    };
    
    StatusByte() = default;
    explicit StatusByte(uint8_t value);

    bool isPowerStageActive() const;
    bool isFault() const;
    bool isGeneratorMode() const;
    bool isAnalogModeActive() const;
    bool isCANEnabled() const;
    bool isCcw() const;
    bool isSafeStopActive() const;
    bool isTOgglebitSet() const;
    
};

}
}
}
}
}

#endif