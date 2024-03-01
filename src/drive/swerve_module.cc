#include "swerve_module.hh"

#include "constants/drive.hh"
#include "swerve_components.hh"
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>
#include <rev/CANSparkBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <string>
#include <units/angle.h>
#include <units/velocity.h>

namespace td::drive {

swerve_module::swerve_module(swerve_module_settings settings_)
    : _drive(settings_.motor_settings.drive_settings,
             settings_.encoder_settings.drive_settings,
             settings_.pid_settings.drive_settings)
    , _azimuth(settings_.motor_settings.azimuth_settings,
               settings_.encoder_settings.azimuth_settings,
               settings_.pid_settings.azimuth_settings)
    , _absolute_encoder(settings_.absolute_encoder_id)
    , _manhattan_offset_from_robot_center(
          settings_.manhattan_offset_from_robot_center) { }

auto
swerve_module::adopt_state(frc::SwerveModuleState state_) -> void {
    _drive.set_target_velocity(state_.speed);
    _azimuth.set_target_angle(state_.angle.Radians());
}

auto
swerve_module::optimize_state(frc::SwerveModuleState state_) const
    -> frc::SwerveModuleState {
    return frc::SwerveModuleState::Optimize(state_, _azimuth.angle());
}

auto
swerve_module::seed_angle_from_absolute_encoder() -> void {
    _azimuth.seed_angle(_absolute_encoder.GetAbsolutePosition().GetValue());
}

auto
swerve_module::target_state() const -> frc::SwerveModuleState {
    return _target_state;
}

auto
swerve_module::mahnattan_offset_from_robot_center() const
    -> frc::Translation2d {
    return _manhattan_offset_from_robot_center;
}

auto
swerve_module::drive_motor() -> swerve_module_drive_motor & {
    return _drive;
}

auto
swerve_module::azimuth_motor() -> swerve_module_azimuth_motor & {
    return _azimuth;
}

auto
swerve_module::log_values() -> void {
    _drive.log_values();
    _azimuth.log_values();

    frc::SmartDashboard::PutNumber(
        std::to_string(_azimuth.controller().GetDeviceId()) + "A-Angle",
        units::angle::radian_t {
            _absolute_encoder.GetAbsolutePosition().GetValue() }
            .value());
}

} // namespace td::drive
