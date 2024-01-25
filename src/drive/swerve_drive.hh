#pragma once

#include "drive/swerve_module.hh"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/angle.h"
#include <AHRS.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace td {

class swerve_drive {
public:
    explicit swerve_drive(std::array<swerve_module_config, 4> module_ids,
                          frc::Translation2d                  offset);

    auto
    get_states_for(units::meters_per_second_t  x_vel,
                   units::meters_per_second_t  y_vel,
                   units::radians_per_second_t angular_velocity)
        -> std::array<frc::SwerveModuleState, 4>;

    auto
    adopt_states(std::array<frc::SwerveModuleState, 4> states) -> void;

    auto
    optimize_and_adopt_states(std::array<frc::SwerveModuleState, 4> states)
        -> void;

    auto
    drive(units::meters_per_second_t  x_vel,
          units::meters_per_second_t  y_vel,
          units::radians_per_second_t angular_velocity) -> void;

    auto
    drive_optimized(units::meters_per_second_t  x_vel,
                    units::meters_per_second_t  y_vel,
                    units::radians_per_second_t angular_velocity) -> void;

    auto
    field_oriented_drive(units::meters_per_second_t  x_vel,
                         units::meters_per_second_t  y_vel,
                         units::radians_per_second_t angular_velocity) -> void;

    auto
    field_oriented_drive_optimized(units::meters_per_second_t  x_vel,
                                   units::meters_per_second_t  y_vel,
                                   units::radians_per_second_t angular_velocity)
        -> void;

    auto
    heading() -> units::radian_t;

    auto
    reset() -> void;

    auto
    log() -> void;

private:
    // Y IS LEFT IN WPILIB

    swerve_module _front_right;
    swerve_module _front_left;
    swerve_module _back_left;
    swerve_module _back_right;

    AHRS gyro;

    frc::SwerveDriveKinematics<4> _kinematics;
};

} // namespace td
