#pragma once

#include "drive/settings.hh"
#include "drive/swerve_module.hh"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include <AHRS.h>
#include <cstdint>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Subsystem.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace td::drive {

uint8_t constexpr FRM_IDX = 0;
uint8_t constexpr FLM_IDX = 1;
uint8_t constexpr BLM_IDX = 2;
uint8_t constexpr BRM_IDX = 3;

enum swerve_drive_orientation_target {
    ROBOT_CENTRIC,
    FIELD_CENTRIC,
    GLOBE_CENTRIC
};

class swerve_drive : public frc2::Subsystem {
public:
    explicit swerve_drive(swerve_drive_settings settings_);

    auto
    calculate_target_states(
        units::velocity::meters_per_second_t          forward_velocity_,
        units::velocity::meters_per_second_t          sideways_velocity_,
        units::angular_velocity::radians_per_second_t angular_velocity_,
        swerve_drive_orientation_target               target_,
        frc::Translation2d center_of_rotation_ = frc::Translation2d {})
        -> swerve_drive_target_states;

    auto
    drive(units::velocity::meters_per_second_t          forward_velocity_,
          units::velocity::meters_per_second_t          sideways_velocity_,
          units::angular_velocity::radians_per_second_t angular_velocity_,
          swerve_drive_orientation_target               target_,
          frc::Translation2d center_of_rotation_ = frc::Translation2d {})
        -> void;

    auto
    seed_azimuth_encoders() -> void;

    auto
    set_module_angles(units::angle::radian_t angle_) -> void;

    auto
    log_values() -> void;

private:
    swerve_module _front_right;
    swerve_module _front_left;
    swerve_module _back_left;
    swerve_module _back_right;

    AHRS _gyro;

    long                          _padding_;
    frc::SwerveDriveKinematics<4> _kinematics;
};

} // namespace td::drive
