#pragma once

#include "constants/util.hh"
#include "drive/settings.hh"
#include <numbers>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

namespace td::drive::k {

constexpr units::radians_per_second_t MAX_ANGULAR_VELOCITY = 180_deg_per_s;
constexpr units::meters_per_second_t  MAX_VELOCITY         = 4_mps;

namespace physical {
constexpr units::meter_t FORWARDS_OFFSET = 10.875_in;
constexpr units::meter_t SIDEWAYS_OFFSET = 10.875_in;

constexpr units::meter_t WHEEL_DIAMETER = 4_in;
constexpr units::meter_t WHEEL_CIRCUMFERENCE =
    WHEEL_DIAMETER * std::numbers::pi;

} // namespace physical

namespace drive {
constexpr double K_P = 0.0;
constexpr double K_I = 0.0;
constexpr double K_D = 0.0;
constexpr double K_F = 0.197;

constexpr double GEAR_RATIO = 6.12 / 1.0;

constexpr double POSITION_CONVERSION_FACTOR =
    physical::WHEEL_CIRCUMFERENCE.value() / GEAR_RATIO;

constexpr double VELOCITY_CONVERSION_FACTOR =
    POSITION_CONVERSION_FACTOR * (1 / 60.0);

constexpr units::second_t RAMP_RATE = 0.5_s;

constexpr util::encoder_settings ENCODER_SETTINGS = {
    .position_conversion_factor = POSITION_CONVERSION_FACTOR,
    .velocity_conversion_factor = VELOCITY_CONVERSION_FACTOR
};

constexpr util::pid_settings PID_SETTINGS { .k_p = K_P,
                                            .k_i = K_I,
                                            .k_d = K_D,
                                            .k_f = K_F };

constexpr util::motor_controller_settings FRONT_RIGHT_CONTROLLER_SETTINGS {
    .id               = 11,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake
};

constexpr util::motor_controller_settings FRONT_LEFT_CONTROLLER_SETTINGS {
    .id               = 21,
    .is_inverted      = false,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

constexpr util::motor_controller_settings BACK_LEFT_CONTROLLER_SETTINGS {
    .id               = 31,
    .is_inverted      = false,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

constexpr util::motor_controller_settings BACK_RIGHT_CONTROLLER_SETTINGS {
    .id               = 41,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

} // namespace drive

namespace azimuth {

constexpr double K_P = 0.35;
constexpr double K_I = 0.0;
constexpr double K_D = 0.0;
constexpr double K_F = 0.0;

constexpr units::radian_t FULL_TURN =
    units::radian_t { 2.0 * std::numbers::pi };

constexpr double GEAR_RATIO = (150.0 / 7.0) / 1.0;

constexpr double POSITION_CONVERSION_FACTOR = FULL_TURN.value() / GEAR_RATIO;

constexpr double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR * 60;

constexpr units::second_t RAMP_RATE = 0.5_s;

constexpr util::pid_settings PID_SETTINGS { .k_p = K_P,
                                            .k_i = K_I,
                                            .k_d = K_D,
                                            .k_f = K_F };

constexpr util::encoder_settings ENCODER_SETTINGS {
    .position_conversion_factor = POSITION_CONVERSION_FACTOR,
    .velocity_conversion_factor = VELOCITY_CONVERSION_FACTOR
};

constexpr util::motor_controller_settings FRONT_RIGHT_CONTROLLER_SETTINGS {
    .id               = 12,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

constexpr util::motor_controller_settings FRONT_LEFT_CONTROLLER_SETTINGS {
    .id               = 22,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

constexpr util::motor_controller_settings BACK_LEFT_CONTROLLER_SETTINGS {
    .id               = 32,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

constexpr util::motor_controller_settings BACK_RIGHT_CONTROLLER_SETTINGS {
    .id               = 42,
    .is_inverted      = true,
    .is_disabled      = false,
    .open_ramp_rate   = RAMP_RATE,
    .closed_ramp_rate = RAMP_RATE,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake

};

} // namespace azimuth

namespace modules {

constexpr swerve_module_settings FRONT_RIGHT_MODULE_SETTINGS {
    .absolute_encoder_id = 13,
    .motor_settings      = {.drive_settings =
                                 drive::FRONT_RIGHT_CONTROLLER_SETTINGS,
                            .azimuth_settings =
                                 azimuth::FRONT_RIGHT_CONTROLLER_SETTINGS},
    .encoder_settings = {
        .drive_settings = drive::ENCODER_SETTINGS,
        .azimuth_settings = azimuth::ENCODER_SETTINGS,
    },

    .pid_settings = {
        .drive_settings = drive::PID_SETTINGS,
        .azimuth_settings = azimuth::PID_SETTINGS
    },
    
    .manhattan_offset_from_robot_center = frc::Translation2d {
        physical::FORWARDS_OFFSET,
        -physical::SIDEWAYS_OFFSET
    }
};

constexpr swerve_module_settings FRONT_LEFT_MODULE_SETTINGS {
    .absolute_encoder_id = 23,
    .motor_settings      = {.drive_settings =
                                 drive::FRONT_LEFT_CONTROLLER_SETTINGS,
                            .azimuth_settings =
                                 azimuth::FRONT_LEFT_CONTROLLER_SETTINGS},
    .encoder_settings = {
        .drive_settings = drive::ENCODER_SETTINGS,
        .azimuth_settings = azimuth::ENCODER_SETTINGS,
    },

    .pid_settings = {
        .drive_settings = drive::PID_SETTINGS,
        .azimuth_settings = azimuth::PID_SETTINGS
    },
    
    .manhattan_offset_from_robot_center = frc::Translation2d {
        physical::FORWARDS_OFFSET,
        physical::SIDEWAYS_OFFSET
    },
};

constexpr swerve_module_settings BACK_LEFT_MODULE_SETTINGS {
    .absolute_encoder_id = 33,
    .motor_settings      = {.drive_settings =
                                 drive::BACK_LEFT_CONTROLLER_SETTINGS,
                            .azimuth_settings =
                                 azimuth::BACK_LEFT_CONTROLLER_SETTINGS},
    .encoder_settings = {
        .drive_settings = drive::ENCODER_SETTINGS,
        .azimuth_settings = azimuth::ENCODER_SETTINGS,
    },

    .pid_settings = {
        .drive_settings = drive::PID_SETTINGS,
        .azimuth_settings = azimuth::PID_SETTINGS
    },
    
    .manhattan_offset_from_robot_center = frc::Translation2d {
        -physical::FORWARDS_OFFSET,
        physical::SIDEWAYS_OFFSET
    }
};

constexpr swerve_module_settings BACK_RIGHT_MODULE_SETTINGS {
    .absolute_encoder_id = 43,
    .motor_settings      = {.drive_settings =
                                 drive::BACK_RIGHT_CONTROLLER_SETTINGS,
                            .azimuth_settings =
                                 azimuth::BACK_RIGHT_CONTROLLER_SETTINGS},
    .encoder_settings = {
        .drive_settings = drive::ENCODER_SETTINGS,
        .azimuth_settings = azimuth::ENCODER_SETTINGS,
    },

    .pid_settings = {
        .drive_settings = drive::PID_SETTINGS,
        .azimuth_settings = azimuth::PID_SETTINGS
    },
    
    .manhattan_offset_from_robot_center = frc::Translation2d {
        -physical::FORWARDS_OFFSET,
        -physical::SIDEWAYS_OFFSET
    }
};

} // namespace modules

constexpr swerve_drive_settings SWERVE_SETTINGS = {
    .front_right_settings = modules::FRONT_RIGHT_MODULE_SETTINGS,
    .front_left_settings  = modules::FRONT_LEFT_MODULE_SETTINGS,
    .back_left_settings   = modules::BACK_LEFT_MODULE_SETTINGS,
    .back_right_settings  = modules::BACK_RIGHT_MODULE_SETTINGS,
};

} // namespace td::drive::k
