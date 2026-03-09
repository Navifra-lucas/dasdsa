#ifndef _NAVIFRA_PDU_H
#define _NAVIFRA_PDU_H

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <nc_cheonil_agent/data/PDUdefinition.h>

namespace NaviFra {
class PDU {
public:
    PDU(const std::unordered_map<std::string, uint16_t>& defaultCoils, const std::unordered_map<std::string, uint16_t>& defaultRegisters);
    PDU();

    const std::unordered_map<std::string, uint16_t>& getCoils() const;
    const std::unordered_map<std::string, uint16_t>& getHoldingRegisters() const;

    void setCoil(const std::string& address, uint16_t value);
    void setHoldingRegister(const std::string& address, uint16_t value);

    uint16_t getCoil(const std::string& key) const;
    uint16_t getHoldingRegister(const std::string& key) const;

    bool loadFromFile(const std::string& filePath);


private:
    std::unordered_map<std::string, uint16_t> coils;
    std::unordered_map<std::string, uint16_t> holdingRegisters;
};
}  // namespace NaviFra
#endif  // NAVIFRA_PDU_H