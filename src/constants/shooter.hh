#pragma once

#include "constants/util.hh"

namespace td::subsystem::shooter::k {

struct shooter_settings {
    util::motor_controller_settings left_controller_settings;
    util::motor_controller_settings right_controller_settings;
    util::encoder_settings          encoder_settings;
    util::pid_settings              pid_settings;
};

constexpr util::pid_settings PID_SETTINGS { .k_p = 0,
                                            .k_i = 0,
                                            .k_d = 0,
                                            .k_f = 0 };

constexpr util::encoder_settings ENCODER_SETTINGS = {
    .position_conversion_factor = 1,
    .velocity_conversion_factor = 1
};

constexpr util::motor_controller_settings RIGHT_CONTROLLER_SETTINGS {
    .id               = 53,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = 0_s,
    .closed_ramp_rate = 0_s,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kCoast

};

constexpr util::motor_controller_settings LEFT_CONTROLLER_SETTINGS {
    .id               = 54,
    .is_inverted      = false,
    .is_disabled      = false,
    .open_ramp_rate   = 0_s,
    .closed_ramp_rate = 0_s,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kCoast

};

constexpr shooter_settings SHOOTER_SETTINGS {
    .left_controller_settings  = LEFT_CONTROLLER_SETTINGS,
    .right_controller_settings = RIGHT_CONTROLLER_SETTINGS,
    .encoder_settings          = ENCODER_SETTINGS,
    .pid_settings              = PID_SETTINGS
};

} // namespace td::subsystem::shooter::k
