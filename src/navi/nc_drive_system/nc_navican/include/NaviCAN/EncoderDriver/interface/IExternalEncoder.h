#ifndef NAVIFRA_NAVICAN_INTERFACE_EXTERNAL_ENCODER_H
#define NAVIFRA_NAVICAN_INTERFACE_EXTERNAL_ENCODER_H

namespace NaviFra
{
    class IExternalEncoder
    {
        public:
            ~IExternalEncoder() = default;
            virtual double getPosition(void) = 0;
            virtual void preset(void) = 0;
            virtual void saveParameter(void) = 0;
    };
} // namespace NaviFra

#endif