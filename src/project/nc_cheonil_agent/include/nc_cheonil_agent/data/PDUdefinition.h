#ifndef _NAVIFRA_PDU_DEFINITION_H
#define _NAVIFRA_PDU_DEFINITION_H

#include <string>

namespace NaviFra {
class PDUdefinition {

public:
    const static std::string PDU_READ_COIL_START;
    const static std::string PDU_READ_COIL_END;
    const static std::string PDU_READ_REGISTER_START;
    const static std::string PDU_READ_REGISTER_END;
    const static std::string PDU_WRITE_COIL_START;
    const static std::string PDU_WRITE_COIL_END;
    const static std::string PDU_WRITE_REGISTER_START;
    const static std::string PDU_WRITE_REGISTER_END;

    const static std::string PDU_WRITE_COIL_FRONT_SAFETY_SCANNER;
    const static std::string PDU_WRITE_COIL_LEFT_SAFETY_SCANNER;
    const static std::string PDU_WRITE_COIL_RIGHT_SAFETY_SCANNER;

    const static std::string PDU_READ_COIL_FRONT_SAFETY_SCANNER;
    const static std::string PDU_READ_COIL_LEFT_SAFETY_SCANNER;
    const static std::string PDU_READ_COIL_RIGHT_SAFETY_SCANNER;

    const static std::string PDU_READ_REGISTER_COMMAND_NUM;
    const static std::string PDU_READ_REGISTER_COMMAND;
    const static std::string PDU_READ_REGISTER_TARGET_NODE;
    const static std::string PDU_READ_REGISTER_X_POSITION;
    const static std::string PDU_READ_REGISTER_Y_POSITION;
    const static std::string PDU_READ_REGISTER_ANGLE_POSITION;
    const static std::string PDU_READ_REGISTER_SPEED_LIMIT;
    const static std::string PDU_READ_REGISTER_JOG_ENABLE;
    const static std::string PDU_READ_REGISTER_JOG_SPEED;
    const static std::string PDU_READ_REGISTER_JOG_ANGLE_SPEED;
    const static std::string PDU_READ_REGISTER_JOG_STEER_DEG;
    const static std::string PDU_READ_REGISTER_SPEED_TYPE;
    const static std::string PDU_READ_REGISTER_NONE_SPEED;
    const static std::string PDU_READ_REGISTER_EMPTY_SPEED;
    const static std::string PDU_READ_REGISTER_LOAD_STATE;
    const static std::string PDU_READ_REGISTER_AUTO_ON;
    const static std::string PDU_READ_REGISTER_BATTERY;
    const static std::string PDU_READ_REGISTER_FORK_UP_DOWN_POSITION;
    const static std::string PDU_READ_REGISTER_FORK_UP_DOWN_COMPLETE;
    const static std::string PDU_READ_REGISTER_TILTING_UP_DOWN;
    const static std::string PDU_READ_REGISTER_FORK_WIDTH;
    const static std::string PDU_READ_REGISTER_PALLET_TOUCH;
    const static std::string PDU_READ_REGISTER_LSC_1;
    const static std::string PDU_READ_REGISTER_LSC_2;
    const static std::string PDU_READ_REGISTER_LSC_3;
    const static std::string PDU_READ_REGISTER_LSC_4;
    const static std::string PDU_READ_REGISTER_CHARGE_STATE;
    const static std::string PDU_READ_REGISTER_JOB_CANCEL;
    const static std::string PDU_READ_REGISTER_PALLET_ID_REMOVE;
    const static std::string PDU_READ_REGISTER_PLC_ALARM;

    const static std::string PDU_WRITE_REGISTER_COMMAND_NUM;
    const static std::string PDU_WRITE_REGISTER_COMMAND;
    const static std::string PDU_WRITE_REGISTER_X_POSITION;
    const static std::string PDU_WRITE_REGISTER_Y_POSITION;
    const static std::string PDU_WRITE_REGISTER_ANGLE_POSITION;
    const static std::string PDU_WRITE_REGISTER_CONFIDENCE;
    const static std::string PDU_WRITE_REGISTER_DRIVE_STATUS;
    const static std::string PDU_WRITE_REGISTER_CURRENT_NODE;
    const static std::string PDU_WRITE_REGISTER_TARGET_NODE;
    const static std::string PDU_WRITE_REGISTER_CURRENT_SPEED;
    const static std::string PDU_WRITE_REGISTER_PATH_SPEED;
    const static std::string PDU_WRITE_REGISTER_ALARM_CODE;
    const static std::string PDU_WRITE_REGISTER_ANGLE_SPEED;
    const static std::string PDU_WRITE_REGISTER_NEXT_NODE;
    const static std::string PDU_WRITE_REGISTER_TRACTION_MOTOR_CURRENT;
    const static std::string PDU_WRITE_REGISTER_STEER_MOTOR_CURRENT;
    const static std::string PDU_WRITE_REGISTER_STEER_DEGREE;
    const static std::string PDU_WRITE_REGISTER_AUTO_CHARGE;
    const static std::string PDU_WRITE_REGISTER_TRACTION_MOTOR_RPM;
    const static std::string PDU_WRITE_REGISTER_STEER_MOTOR_RPM;
    const static std::string PDU_WRITE_REGISTER_FORK_UP_DOWN_POSITION;
    const static std::string PDU_WRITE_REGISTER_FORK_UP_DOWN_CMD;
    const static std::string PDU_WRITE_REGISTER_TILTING_UP_DOWN_CMD;
    const static std::string PDU_WRITE_REGISTER_FORK_WIDTH_CMD;
    const static std::string PDU_WRITE_REGISTER_LIGHT_CMD;
    const static std::string PDU_WRITE_REGISTER_LSC_FIELD;
    const static std::string PDU_WRITE_REGISTER_HAS_JOB;
    const static std::string PDU_WRITE_REGISTER_HAS_ID;

};
}  // namespace Navifra
#endif