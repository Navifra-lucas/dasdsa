#ifndef NAVIFRA_MODBUS_PORT_H
#define NAVIFRA_MODBUS_PORT_H

#include <stdint.h>

#include <functional>
#include <vector>

namespace NaviFra {

class ModbusPort {
public:
    virtual ~ModbusPort() = default;

    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void sendFrame(const std::vector<uint8_t>& pdu, std::function<void()> onWriteComplete) = 0;
    virtual void asyncWrite(const std::vector<uint8_t>& data, std::function<void()> onComplete) = 0;
    virtual void asyncRead(std::size_t size, std::function<void(const std::vector<uint8_t>&)> onReceive) = 0;
    virtual bool isOpen() const = 0;
};
}  // namespace NaviFra

#endif  // NAVIFRA_MODBUS_PORT_H