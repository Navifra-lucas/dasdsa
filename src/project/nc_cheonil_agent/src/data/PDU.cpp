#include "nc_cheonil_agent/nc_cheonil_agent_pch.h"

#include <nc_cheonil_agent/data/PDU.h>
#include <nc_cheonil_agent/data/PDUdefinition.h>

using namespace NaviFra;
PDU::PDU()
{
    setCoil(PDUdefinition::PDU_READ_COIL_START, 0);    // 0x00
    setCoil(PDUdefinition::PDU_READ_COIL_END, 10);     // 0x0A
    setCoil(PDUdefinition::PDU_WRITE_COIL_START, 0);   // 0x00
    setCoil(PDUdefinition::PDU_WRITE_COIL_END, 10);    // 0x0A

    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_START, 0);   // 0x00
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_END, 29);    // 0x1D

    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_START, 0);  // 0x00
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_END, 28);   // 0x1C

    // WRITE_COIL
    setCoil(PDUdefinition::PDU_WRITE_COIL_FRONT_SAFETY_SCANNER, 0);  // 0x00
    setCoil(PDUdefinition::PDU_WRITE_COIL_LEFT_SAFETY_SCANNER, 1);   // 0x01
    setCoil(PDUdefinition::PDU_WRITE_COIL_RIGHT_SAFETY_SCANNER, 2);  // 0x02

    // READ_COIL
    setCoil(PDUdefinition::PDU_READ_COIL_FRONT_SAFETY_SCANNER, 0);   // 0x00
    setCoil(PDUdefinition::PDU_READ_COIL_LEFT_SAFETY_SCANNER, 1);    // 0x01
    setCoil(PDUdefinition::PDU_READ_COIL_RIGHT_SAFETY_SCANNER, 2);   // 0x02

    // READ REGISTER
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND_NUM, 0);            // 0x00
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_COMMAND, 1);                // 0x01
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_TARGET_NODE, 2);            // 0x02
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_X_POSITION, 3);             // 0x03
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_Y_POSITION, 4);             // 0x04
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_ANGLE_POSITION, 5);         // 0x05
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_SPEED_LIMIT, 6);            // 0x06
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_JOG_ENABLE, 7);             // 0x07
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_JOG_SPEED, 8);              // 0x08
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_JOG_ANGLE_SPEED, 9);        // 0x09
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_JOG_STEER_DEG, 10);         // 0x0A
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_SPEED_TYPE, 11);            // 0x0B
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_NONE_SPEED, 12);            // 0x0C
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_EMPTY_SPEED, 13);           // 0x0D
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_LOAD_STATE, 14);            // 0x0E
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_AUTO_ON, 15);               // 0x0F
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_BATTERY, 16);               // 0x10
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_FORK_UP_DOWN_POSITION, 17); // 0x11
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_FORK_UP_DOWN_COMPLETE, 18); // 0x12
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_TILTING_UP_DOWN, 19);       // 0x13
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_FORK_WIDTH, 20);            // 0x14
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_PALLET_TOUCH, 21);                 // 0x19
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_LSC_1, 22);                 // 0x15
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_LSC_2, 23);                 // 0x16
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_LSC_3, 24);                 // 0x17
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_LSC_4, 25);                 // 0x18
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_CHARGE_STATE, 26);          // 0x1A
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_JOB_CANCEL, 27);            // 0x1B
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_PALLET_ID_REMOVE, 28);      // 0x1C
    setHoldingRegister(PDUdefinition::PDU_READ_REGISTER_PLC_ALARM, 29);             // 0x1D

    // WRITE REGISTER
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_COMMAND_NUM, 0);               // 0x00
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_COMMAND, 1);                   // 0x01
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_X_POSITION, 2);                // 0x02
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_Y_POSITION, 3);                // 0x03
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_ANGLE_POSITION, 4);            // 0x04
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_CONFIDENCE, 5);                // 0x05
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_DRIVE_STATUS, 6);              // 0x06
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_CURRENT_NODE, 7);              // 0x07
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_TARGET_NODE, 8);               // 0x08
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_CURRENT_SPEED, 9);             // 0x09
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_PATH_SPEED, 10);               // 0x0A
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_ALARM_CODE, 11);               // 0x0B
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_ANGLE_SPEED, 12);              // 0x0C
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_NEXT_NODE, 13);                // 0x0D
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_TRACTION_MOTOR_CURRENT, 14);   // 0x0E
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_STEER_MOTOR_CURRENT, 15);      // 0x0F
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_STEER_DEGREE, 16);             // 0x10
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_AUTO_CHARGE, 17);              // 0x11
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_TRACTION_MOTOR_RPM, 18);       // 0x12
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_STEER_MOTOR_RPM, 19);          // 0x13
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_FORK_UP_DOWN_POSITION, 20);    // 0x14
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_FORK_UP_DOWN_CMD, 21);         // 0x15
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_TILTING_UP_DOWN_CMD, 22);      // 0x16
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_FORK_WIDTH_CMD, 23);           // 0x17
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_LIGHT_CMD, 24);                // 0x18
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_LSC_FIELD, 25);                // 0x19
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_HAS_JOB, 26);                  // 0x1A
    setHoldingRegister(PDUdefinition::PDU_WRITE_REGISTER_HAS_ID, 27);                   // 0x1B
}

PDU::PDU(const std::unordered_map<std::string, uint16_t>& defaultCoils, const std::unordered_map<std::string, uint16_t>& defaultRegisters)
    : coils(defaultCoils)
    , holdingRegisters(defaultRegisters)
{
    PDU();
}

const std::unordered_map<std::string, uint16_t>& PDU::getCoils() const
{
    return coils;
}
const std::unordered_map<std::string, uint16_t>& PDU::getHoldingRegisters() const
{
    return holdingRegisters;
}
uint16_t PDU::getCoil(const std::string& key) const
{
    auto it = coils.find(key);
    if (it != coils.end()) {
        return it->second;
    }
    else {
        throw std::runtime_error("Coil key not found");
    }
}
uint16_t PDU::getHoldingRegister(const std::string& key) const
{
    auto it = holdingRegisters.find(key);
    if (it != holdingRegisters.end()) {
        return it->second;
    }
    else {
        throw std::runtime_error("Holding register key not found");
    }
}
void PDU::setCoil(const std::string& address, uint16_t value)
{
    coils[address] = value;
}
void PDU::setHoldingRegister(const std::string& address, uint16_t value)
{
    holdingRegisters[address] = value;
}
bool PDU::loadFromFile(const std::string& filePath)
{
    try {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            return false;
        }
        std::string line;
        while (std::getline(file, line)) {
            // 주석인 경우 무시 (줄이 비어 있거나 '#'으로 시작하는 경우)
            if (line.empty() || line[0] == '#' || line[0] == '/' && line[1] == '/') {
                continue;
            }
            std::istringstream iss(line);
            std::string address;
            uint16_t value;
            uint16_t isCoil;
            if (!(iss >> address >> std::hex >> value >> isCoil)) {
                std::cerr << "Failed to read address/value from file: " << filePath << std::endl;
                return false;
            }
            if (isCoil) {
                setCoil(address, value);
            }
            else {
                setHoldingRegister(address, value);
            }
        }
    }
    catch (std::exception& ex) {
        LOG_ERROR("loadFromFile error %s", ex.what());
        return false;
    }
    return true;
}