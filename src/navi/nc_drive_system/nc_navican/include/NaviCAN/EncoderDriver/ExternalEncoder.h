#ifndef NAVIFRA_NAVICAN_EXTERNAL_ENCODER_H
#define NAVIFRA_NAVICAN_EXTERNAL_ENCODER_H

#include "NaviCAN/NaviCANDriver.h"
#include "NaviCAN/EncoderDriver/interface/IExternalEncoder.h"

namespace NaviFra {

class ExternalEncoder : public IExternalEncoder 
{
public:
    ExternalEncoder(std::shared_ptr<NaviCANDriver> driver)
        : driver_(driver) { }

    double getPosition(void) override;
    void preset(void) override;
    void saveParameter(void) override;

private:

    std::shared_ptr<NaviCANDriver> driver_;

    enum {
        save_parameters_index = 0x1010,
        preset_value_index = 0x6003,
        position_value_index = 0x6004,
    };
};

}  // namespace NaviFra

#endif  // NAVIFRA_EXTERNALENCODER_H
