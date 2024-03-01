#pragma once

#include "constants/util.hh"
#include "drive/settings.hh"
#include "drive/swerve_components.hh"
#include <cstdint>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/AnalogEncoder.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/MotorFeedbackSensor.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

namespace td::drive {

class swerve_module {
public:
    explicit swerve_module(swerve_module_settings settings_);

    auto
    adopt_state(frc::SwerveModuleState state_) -> void;

    [[nodiscard]] auto
    optimize_state(frc::SwerveModuleState state_) const
        -> frc::SwerveModuleState;

    auto
    seed_angle_from_absolute_encoder() -> void;

    [[nodiscard]] auto
    target_state() const -> frc::SwerveModuleState;

    [[nodiscard]] auto
    mahnattan_offset_from_robot_center() const -> frc::Translation2d;

    [[nodiscard]] auto
    drive_motor() -> swerve_module_drive_motor &;

    [[nodiscard]] auto
    azimuth_motor() -> swerve_module_azimuth_motor &;

    auto
    log_values() -> void;

private:
    swerve_module_drive_motor   _drive;
    swerve_module_azimuth_motor _azimuth;

    frc::SwerveModuleState _target_state;

    ctre::phoenix6::hardware::CANcoder _absolute_encoder;

    frc::Translation2d _manhattan_offset_from_robot_center;
};

} // namespace td::drive
