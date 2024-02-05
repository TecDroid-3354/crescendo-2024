#pragma once

#include "drive/swerve_module.hh"
#include <AHRS.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Subsystem.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace td {

class swerve_drive : public frc2::Subsystem {
public:
    explicit swerve_drive(std::array<swerve_module_config, 4> module_ids,
                          frc::Translation2d                  offset);

    auto
    get_robot_centric_states_for(units::meters_per_second_t  x_vel,
                                 units::meters_per_second_t  y_vel,
                                 units::radians_per_second_t angular_velocity)
        -> std::array<frc::SwerveModuleState, 4>;

    auto
    get_field_centric_states_for(units::meters_per_second_t  x_vel,
                                 units::meters_per_second_t  y_vel,
                                 units::radians_per_second_t angular_velocity)
        -> std::array<frc::SwerveModuleState, 4>;

    auto
    adopt_state_array(std::array<frc::SwerveModuleState, 4> states) -> void;

    auto
    drive(units::meters_per_second_t  x_vel,
          units::meters_per_second_t  y_vel,
          units::radians_per_second_t angular_velocity) -> void;

    auto
    field_oriented_drive(units::meters_per_second_t  x_vel,
                         units::meters_per_second_t  y_vel,
                         units::radians_per_second_t angular_velocity) -> void;

    auto
    sync_modules_with_cancoders() -> void;

    /**
     * zeroes out the physical position of all moduels
     */
    auto
    zero_modules() -> void;

    /**
     * Returns the robot's heading angle (z-axis)
     */
    auto
    heading() -> units::radian_t;

    /**
     * Resets the azimuth and drive encoders along with the absolute encoders as
     * well as the gyro's angle
     */
    auto
    full_reset() -> void;

    /**
     * Resets the azimuth and drive encoders as well as the gyro's angle
     */
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

    AHRS _gyro;

    frc::SwerveDriveKinematics<4> _kinematics;
};

} // namespace td
