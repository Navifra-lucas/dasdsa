#ifndef NC_DATA_HANDLER_H
#define NC_DATA_HANDLER_H

#include <array>
#include <string> 
#include <map> 

enum class INPUT
{
    EQUIPMENT1 = 0,
    EQUIPMENT2, 
    EQUIPMENT3, 
    EQUIPMENT4, 
    OPTION1,
    OPTION2,
    OPTION3, 
    OPTION4, 
    SAFETY,
    SPARE1,
    BUTTON,
    SPARE2, 
    SPARE3, 
    STATE_INFO,
    SPARE4,
    SPARE5, 
    SENSOR1,
    SENSOR2,
    SPARE6,
    PIN,
    LIFT, 
    SPARE8, 
    LIFT_TURN, 
    SPARE9,
    SPARE10, 
    CONVEYOR1, 
    CONVEYOR2, 
    CONVEYOR3, 
    SPARE11, 
    SPARE12, 
    FRONT_OSSD,
    REAR_OSSD, 
    IO1 = 53,
    IO2,
    IO3,
    IO4,
    IO5,
    IO6,
    IO7,
    VERSION_MAJOR = 60, 
    VERSION_MINOR, 
    VERSION_PATCH,
    VERSION_RESERVED,
    CHARGE_RELAY,
};

enum class OUTPUT {
    EQUIPMENT1 = 0,
    EQUIPMENT2, 
    EQUIPMENT3, 
    EQUIPMENT4, 
    OPTION1,
    OPTION2,
    OPTION3, 
    OPTION4, 
    STATE_INFO,
    COMMAND,
    COMMAND2,
    FRONT_OSSD,
    REAR_OSSD,
    LIFT = 20, 
    LIFT_TURN, 
    CONVEYOR,
    UPPER,
    PIN,
    CHARGE,
};

enum class SAFETY {
    FRONT_BUMPPER = 0,
    EMERGENCY, 
    OSSD_SIGNAL, 
    STO_SIGNAL, 
    BRAKE_RELEASE,
    MANUAL_CHARGE, 
    QUICK_STOP_SIGNAL,
    REAR_BUMPER,
};

enum class BUTTON {
    RESET = 0, 
    BRAKE, 
    MODE_SELECTE_1,
    MODE_SELECTE_2,
    IPC_OFF = 7,
};

enum class HW_STATE {
    INPUT_YELLOW = 0, 
    INPUT_GREEN, 
    INPUT_POWER, 
    INPUT_SELECT, 
    RUNNING,
    MOVING,
};

enum class SENSOR {
    FRONT_DOCK = 0,
    REAR_DOCK, 
    FRONT_MAGNET_SENSOR, 
    REAR_MAGNET_SENSOR, 
    LIFT_ALARM, 
    FRONT_DOCK_2,
    DRIVE_POWER_RELAY_CHECK_SIGNAL, 
    REAR_CHARGE_RELAY_CHECK_SIGNAL, 
    FRONT_CHARGE_RELAY_CHECK_SIGNAL, 
};

enum class LIFT { 
    DOWN_1 = 0,
    DOWN_2,
    UP_1, 
    UP_2,
};

enum class LIFT_TURN {
    UP_FEEDBACK, 
    DOWN_FEEDBACK, 
    FRONT_TURN_TABLE,
    REAR_TURN_TABLE,
    LIFT_ALARM
};

enum class CHARGE {
    CHARGE_RELAY,
};

enum class CONVEYOR {
    FWD_BUTTON = 0,
    BWD_BUTTON, 
    FRONT_OBJECT, 
    REAR_OBJECT, 
    REAR_STOPPER_UP, 
    REAR_STOPPER_DOWN,
    FRONT_STOPPER_UP,
    FRONT_STOPPER_DOWN,
    CONVEYOR_ALARM, 
    ERROR1, 
    ERROR2, 
    ERROR3, 
    ERROR4, 
    ERROR5, 
    ERROR6, 
    ERROR7, 
    LOAD_COMPLETE_FEEDBACK, 
    UNLOAD_COMPLETE_FEEDBACK,
};

enum class PIN {
    FRONT_UP_PX = 0,
    FRONT_DOWN_PX,
    REAR_UP_PX,
    REAR_DOWN_PX,
    FRONT_MOTOR_ON,
    REAR_MOTOR_ON,
    FRONT_MOTOR_ALARM,
    REAR_MOTOR_ALARM,
};

enum class STATE {
    // 
};

enum class COMMAND {
    TABLE_BRAKE_RELEASE = 0, 
    BRAKE, 
    CHARGE_RELAY_REAR, 
    CHARGE_RELAY_FRONT,
    STO_CLEAR,
    MANUAL_MODE_ON, 
    REMOTE_RESET, 
    LIFT_BRAKE
};

enum class COMMAND2 {
    QUICK_STOP = 0, 
    BUMPER_CLEAR,
    OSSD_BYPASS,
    LIDAR_OSSD_ENABLE,
    STO_CHECK
};


static constexpr size_t IO_DATA_SIZE = 64;

namespace NaviFra{

struct PlcInfoData {
    // 안전
    bool front_bumper;
    bool rear_bumper;
    bool emergency_button;
    bool ossd_signal;
    bool sto_signal;
    bool brake_feedback;
    bool manual_charge_on;
    bool quick_stop_status;

    // 버튼
    bool reset_switch;
    bool brake_switch;
    bool mode_select_1;
    bool mode_select_2;

    // 센서 / 릴레이
    bool drive_power_relay;
    bool rear_charge_relay;

    // 핀 / 리프트
    bool front_up_px;
    bool front_down_px;
    bool rear_up_px;
    bool rear_down_px;
    bool front_motor_on;
    bool rear_motor_on;
    bool front_motor_alarm;
    bool rear_motor_alarm;

    // 원시 IO
    uint8_t io_ib0;
    uint8_t io_qb6;
    uint8_t io_qb12;
    uint8_t io_qb18;
    uint8_t io_ib24;
    uint8_t io_ib25;
    uint8_t io_qb26;

    // 버전
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
};

class DataHandler
{
public:

    DataHandler();
    virtual ~DataHandler();
    
private:
    std::map<int, u_int8_t> input_;
    std::map<int, u_int8_t> output_;

public:
    std::map<int, u_int8_t> getInput() { return input_; }
    std::map<int, u_int8_t> getOutput() {return output_; }
    
    std::array<u_int8_t, IO_DATA_SIZE> getInputArray() const;
    std::array<u_int8_t, IO_DATA_SIZE> getOutputArray() const;

    void setInputArray(const std::array<u_int8_t, IO_DATA_SIZE>& input);
    void setOutputArray(const std::array<u_int8_t, IO_DATA_SIZE>& output);

    bool getSafetyBit(SAFETY safetyBit); 
    bool getButtonBit(BUTTON buttonBit); 
    bool getHWStateBit(HW_STATE stateBit); 
    bool getSensorBit(SENSOR sensorBit); 
    bool getPinBit(PIN pinBit);
    bool getLiftBit(LIFT lifteBit); 
    bool getLiftTurnBit(LIFT_TURN liftTurnBit); 
    bool getConveyorBit(CONVEYOR conveyorBit); 
    bool getChargeRelayBit(CHARGE chargeRelayBit);

    void setCommandBit(COMMAND cmd, bool value);
    void setCommandBit2(COMMAND2 cmd, bool value);
    
    bool getCommandBit(COMMAND cmd);   // OUTPUT::COMMAND의 특정 비트 읽기
    bool getCommandBit2(COMMAND2 cmd); // OUTPUT::COMMAND2의 특정 비트 읽기
    
    // enum을 사용한 타입 안전 메서드들
    u_int8_t getInputValue(INPUT input_type);
    u_int8_t getOutputValue(OUTPUT output_type);
    void setInputValue(INPUT input_type, u_int8_t value);
    void setOutputValue(OUTPUT output_type, u_int8_t value);

    PlcInfoData getAllPlcInfo();
private: 
    std::array<u_int8_t, IO_DATA_SIZE> mapToArray(const std::map<int, u_int8_t>& map) const;
    std::map<int, u_int8_t> arrayToMap(const std::array<u_int8_t, IO_DATA_SIZE>& array);
};
} // namespace NaviFra

#endif  // NC_DATA_HANDLER_H