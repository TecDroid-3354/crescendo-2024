#pragma once

#include "constants/util.hh"

namespace td::subsystem::indexer::k {

struct indexer_settings {
    util::motor_controller_settings controller_settings;
    util::encoder_settings          encoder_settings;
    util::pid_settings              pid_settings;
};

constexpr util::encoder_settings ENCODER_SETTINGS {
    .position_conversion_factor = 1,
    .velocity_conversion_factor = 1
};

constexpr util::pid_settings PID_SETTINGS { .k_p = 0,
                                            .k_i = 0,
                                            .k_d = 0,
                                            .k_f = 0 };

constexpr util::motor_controller_settings CONTROLLER_SETTINGS {
    .id               = 52,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = 0_s,
    .closed_ramp_rate = 0_s,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake
};

constexpr indexer_settings INDEXER_SETTINGS { .controller_settings =
                                                  CONTROLLER_SETTINGS };

} // namespace td::subsystem::indexer::k
