#ifndef NAVIFRA_MODBUS_CONSTANTS_H
#define NAVIFRA_MODBUS_CONSTANTS_H

#include <stdint.h>

#include <cstddef>

namespace NaviFra {

constexpr size_t MBAP_HEADER_SIZE = 7;  // MBAP 헤더 크기: 7 바이트

enum class FunctionCode : uint8_t
{
    READ_COILS = 0x01,
    READ_DISCRETE_INPUTS = 0x02,
    READ_HOLDING_REGISTERS = 0x03,
    READ_INPUT_REGISTERS = 0x04,
    WRITE_SINGLE_COIL = 0x05,
    WRITE_SINGLE_REGISTER = 0x06,
    WRITE_MULTIPLE_COILS = 0x0F,
    WRITE_MULTIPLE_REGISTERS = 0x10
};
}  // namespace NaviFra

#endif  // NAVIFRA_MODBUS_CONSTANTS_H
