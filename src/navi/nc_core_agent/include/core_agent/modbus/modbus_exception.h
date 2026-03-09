
#ifndef NAVIFRA_MODBUSEXCEPTION_H
#define NAVIFRA_MODBUSEXCEPTION_H

#include <stdexcept>
#include <string>

namespace NaviFra {
class ModbusException : public std::runtime_error {
public:
    explicit ModbusException(const std::string& message)
        : std::runtime_error("Modbus Error: " + message)
    {
    }
};
}  // namespace NaviFra

#endif  // NAVIFRA_MODBUSEXCEPTION_H
