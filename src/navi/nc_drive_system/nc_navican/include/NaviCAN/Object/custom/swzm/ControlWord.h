#ifndef NAVICAN_OBJECT_SWZM_CONTROLWORD_HPP
#define NAVICAN_OBJECT_SWZM_CONTROLWORD_HPP

#include "NaviCAN/Object/BitField.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {
namespace Custom {
namespace swzm {

    class ControlWord : public NaviFra::NaviCAN::Object::BitField<uint16_t>
    {
        public:
        // 비트 위치 정의
        enum Bits : uint16_t {
            DRIVE_ENABLE = 0,               // 0 : 드라이브 비활성화, 1 : 드라이브 활성화
            MAIN_CONTACTOR_ENABLE = 1,      // 0 : 메인 컨택터 OFF -> DC링크 차단, 1 : 메인 컨택터 ON -> DC링크 연결, 출력 전류 가능
            BRAKE_RELEASE = 2,              // 0 : Brake 출력 OFF -> 브레이크 잠김, 1 : Brake 출력 ON -> 브레이크 해제(열림)
            DIG_OUTPUT_1 = 3,               // 0 : 출력 OFF, 1 : 출력 ON
            DIG_OUTPUT_2 = 4,               // 0 : 출력 OFF, 1 : 출력 ON
            DIG_OUTPUT_3 = 5,               // 0 : 출력 OFF, 1 : 출력 ON
            DIG_OUTPUT_4 = 6,               // 0 : 출력 OFF, 1 : 출력 ON
            SAFE_STOP_FUNCTION = 7,         // 0 : 정상 운전, 1 : Safe Stop 동작 중, CAN모드에서만 유효, 활성화 시 속도 명령을 0rpm으로, 감속률을 EE_DcelSafeStopMax로 강제 설정
            // Bit8 ~ Bit14: Not used       // 사용되지 않음
            RESET_FAULT = 15                // 오류 리셋 시 "rising edge" 필요, 오류 발생 후 이 비트를 0 -> 1로 토글해야 해제
            //
        };
        ControlWord() = default;
        explicit ControlWord(uint16_t value);

        void enableDrive(bool value);
        void enableMainContactor(bool value);
        void enableBrake(bool value);
        void enableSafeStop(bool value);
        void faultReset();
    };
}
}
}
}
}

#endif