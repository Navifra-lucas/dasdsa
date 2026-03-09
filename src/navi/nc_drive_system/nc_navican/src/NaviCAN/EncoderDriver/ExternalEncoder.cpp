#include "NaviCAN/EncoderDriver/ExternalEncoder.h"

using namespace NaviFra;

double ExternalEncoder::getPosition(void)
{
    return static_cast<double>(driver_->universal_get_value<int32_t>(position_value_index, 0x00));
}

void ExternalEncoder::saveParameter()
{
    int32_t save_value = 0x65766173;// "save" in ASCII
    driver_->universal_set_value<int32_t>(save_parameters_index, 0x01,save_value);
}

void ExternalEncoder::preset(void)
{
    driver_->universal_set_value<int32_t>(preset_value_index, 0x00, 0);
    saveParameter(); 
}