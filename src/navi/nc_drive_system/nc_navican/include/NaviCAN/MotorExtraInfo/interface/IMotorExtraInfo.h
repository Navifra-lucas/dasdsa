#ifndef NAVIFRA_NAVICAN_INTERFACE_IMOTOR_EXTRA_INFO_H
#define NAVIFRA_NAVICAN_INTERFACE_IMOTOR_EXTRA_INFO_H

#include <string>
namespace NaviFra
{
    class IMotorExtraInfo 
    {
    public:
        virtual ~IMotorExtraInfo() = default;
        
        virtual double getVoltage() = 0;
        virtual double getCurrent() = 0;
        virtual uint32_t getErrorCode() = 0;
        virtual std::string getErrorMessage() = 0;
        virtual uint16_t getSTOCode() = 0;

    };
} // namespace NaviFra

#endif