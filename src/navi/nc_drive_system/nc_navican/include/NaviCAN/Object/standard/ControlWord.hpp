#ifndef NAVICAN_NAVICAN_OBJECT_CONTROLWORD_HPP
#define NAVICAN_NAVICAN_OBJECT_CONTROLWORD_HPP

#include "NaviCAN/Object/BitField.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Standard {
    class ControlWord : public NaviFra::NaviCAN::Object::BitField<uint16_t>
    {
        public:
        // 비트 위치 정의
        enum Bits : uint16_t {
            SWITCH_ON = 0,               // 전원 스위치 켜기
            ENABLE_VOLTAGE = 1,          // 전압 활성화
            QUICK_STOP = 2,              // 빠른 정지 (1=비활성, 0=활성)
            ENABLE_OPERATION = 3,        // 동작 활성화
            OPERATION_MODE_SPECIFIC0 = 4,
            OPERATION_MODE_SPECIFIC1 = 5,
            OPERATION_MODE_SPECIFIC2 = 6,
            FAULT_RESET = 7,             // 오류 리셋 (0→1 전환으로 트리거)
            HALT = 8                     // 정지 (1=정지, 0=정상 동작)
            // 9-15: 제조사 정의 또는 예약됨
        };
        ControlWord() = default;
        explicit ControlWord(uint16_t value);

        // CiA402
        void shutdown();
        void switchOn();
        void disableVoltage();
        void QuickStop();
        void disableOperation();
        void enableOperation();
        void faultReset(bool value);

    };
}
}
}
}


#endif