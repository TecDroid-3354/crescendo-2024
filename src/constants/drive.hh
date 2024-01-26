#pragma once

#include "drive/swerve_module.hh"
#include "units/angle.h"
#include "units/base.h"
#include "units/length.h"
#include "units/velocity.h"
#include <numbers>
#include <units/angular_velocity.h>

namespace td::k::swerve {

constexpr units::meter_t robot_width  = 1_in;
constexpr units::meter_t robot_length = 1_in;

namespace module {
constexpr units::meter_t sideways_offset = 14_in;
constexpr units::meter_t forwards_offset = 14.5_in;

constexpr units::meter_t wheel_diameter = 4_in;
constexpr units::meter_t wheel_circumference =
    wheel_diameter * std::numbers::pi;

constexpr swerve_module_config front_right_ids { 21, 22, 23, false };
constexpr swerve_module_config front_left_ids { 31, 32, 33, true };
constexpr swerve_module_config back_left_ids { 41, 42, 43, true };
constexpr swerve_module_config back_right_ids { 11, 12, 13, false };

constexpr std::array<swerve_module_config, 4> module_ids { front_right_ids,
                                                           front_left_ids,
                                                           back_left_ids,
                                                           back_right_ids };

} // namespace module

namespace azimuth {
constexpr double K_P = 0.2;
constexpr double K_I = 0.0;
constexpr double K_D = 0.0;
constexpr double K_F = 0.0;

constexpr units::radian_t full_turn =
    units::radian_t { 2.0 * std::numbers::pi };

constexpr double gear_ratio                 = (150.0 / 7.0) / 1.0;
constexpr double position_conversion_factor = full_turn.value() / gear_ratio;
constexpr double velocity_conversion_factor = position_conversion_factor * 60;

constexpr units::radians_per_second_t angular_velocity { std::numbers::pi
                                                         * 3.0 };
} // namespace azimuth

namespace drive {
constexpr double K_P = 0.0;
constexpr double K_I = 0.0;
constexpr double K_D = 0.0;
constexpr double K_F = 0.197;

constexpr double gear_ratio = 6.12 / 1.0;
constexpr double position_conversion_factor =
    module::wheel_circumference.value() / gear_ratio;
constexpr double velocity_conversion_factor =
    position_conversion_factor * (1 / 60.0);

constexpr units::meters_per_second_t velocity = 4_mps;

} // namespace drive

} // namespace td::k::swerve
