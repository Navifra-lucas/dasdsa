#ifndef NAVIFRA_SENSOR_OFFSET_H_
#define NAVIFRA_SENSOR_OFFSET_H_

namespace NaviFra {
struct ExtrinsicParameter_t {
    float f_x_m;
    float f_y_m;
    float f_z_m;

    float f_roll_deg;
    float f_pitch_deg;
    float f_yaw_deg;

    float f_roll_rad;
    float f_pitch_rad;
    float f_yaw_rad;

    ExtrinsicParameter_t()
    {
        f_x_m = 0.f;
        f_y_m = 0.f;
        f_z_m = 0.f;

        f_roll_deg = 0.f;
        f_pitch_deg = 0.f;
        f_yaw_deg = 0.f;

        f_roll_rad = 0.f;
        f_pitch_rad = 0.f;
        f_yaw_rad = 0.f;
    }
};

}  // namespace NaviFra

#endif