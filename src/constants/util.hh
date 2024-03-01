#pragma once

#include <cstdint>
#include <units/time.h>

namespace td::util {

struct motor_controller_settings {
    uint8_t               id               = 0;
    bool                  is_inverted      = false;
    bool                  is_disabled      = false;
    units::time::second_t open_ramp_rate   = 0.0_s;
    units::time::second_t closed_ramp_rate = 0.0_s;
};

struct encoder_settings {
    double position_conversion_factor = 1.0;
    double velocity_conversion_factor = 1.0;
};

struct pid_settings {
    double k_p = 0.0;
    double k_i = 0.0;
    double k_d = 0.0;
    double k_f = 0.0;
};

} // namespace td::util
