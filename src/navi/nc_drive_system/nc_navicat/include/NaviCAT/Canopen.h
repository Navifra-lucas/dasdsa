#ifndef NAVIFRA_NAVICAN_CANOPEN_H
#define NAVIFRA_NAVICAN_CANOPEN_H

namespace NaviFra {
namespace NaviCAT {
namespace Canopen {

// DS402 상태 및 제어 정의
namespace DS402 {
    constexpr uint16_t STATUS_SWITCH_ON_DISABLED = 0x0040;
    constexpr uint16_t STATUS_READY_TO_SWITCH_ON = 0x0021;
    constexpr uint16_t STATUS_SWITCHED_ON = 0x0023;
    constexpr uint16_t STATUS_OPERATION_ENABLED = 0x0027;
    constexpr uint16_t STATUS_FAULT = 0x0008;

    constexpr uint16_t CTRL_SHUTDOWN = 0x0006;
    constexpr uint16_t CTRL_SWITCH_ON = 0x0007;
    constexpr uint16_t CTRL_ENABLE_OPERATION = 0x000F;
    constexpr uint16_t CTRL_DISABLE_VOLTAGE = 0x0000;
    constexpr uint16_t CTRL_QUICK_STOP = 0x0002;
    constexpr uint16_t CTRL_DISABLE_OPERATION = 0x0007;
    constexpr uint16_t CTRL_FAULT_RESET = 0x0080;
}

// 동작 모드
namespace OperationMode {
    constexpr int8_t PROFILE_POSITION = 1;
    constexpr int8_t PROFILE_VELOCITY = 3;
    constexpr int8_t HOMING = 6;
    constexpr int8_t CYCLIC_SYNC_POSITION = 8;
    constexpr int8_t CYCLIC_SYNC_VELOCITY = 9;
}

}
}
}

#endif