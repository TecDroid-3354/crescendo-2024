#include "swerve_components.hh"

#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/SparkRelativeEncoder.h"
#include <iostream>
#include <numbers>
#include <string>

// Driver motors

namespace td::drive {
swerve_module_drive_motor::swerve_module_drive_motor(
    util::motor_controller_settings motor_settings_,
    util::encoder_settings          encoder_settings_,
    util::pid_settings              pid_settings_)
    : _controller(motor_settings_.id, rev::CANSparkMax::MotorType::kBrushless)
    , _encoder(_controller.GetEncoder())
    , _pid_controller(_controller.GetPIDController())
    , _target_velocity(0_mps)
    , _disabled(motor_settings_.is_disabled) {
    _controller.SetInverted(motor_settings_.is_inverted);
    _controller.SetOpenLoopRampRate(motor_settings_.open_ramp_rate.value());
    _controller.SetClosedLoopRampRate(motor_settings_.closed_ramp_rate.value());
    _controller.SetIdleMode(motor_settings_.idle_mode);

    _encoder.SetPositionConversionFactor(
        encoder_settings_.position_conversion_factor);
    _encoder.SetVelocityConversionFactor(
        encoder_settings_.velocity_conversion_factor);

    _pid_controller.SetP(pid_settings_.k_p);
    _pid_controller.SetI(pid_settings_.k_i);
    _pid_controller.SetD(pid_settings_.k_d);
    _pid_controller.SetFF(pid_settings_.k_f);
    _pid_controller.SetOutputRange(-1.0, 1.0);
}

auto
swerve_module_drive_motor::set_target_velocity(
    units::velocity::meters_per_second_t velocity_) -> void {
    if (_disabled) {
        return;
    }

    _pid_controller.SetReference(velocity_.value(),
                                 rev::CANSparkMax::ControlType::kVelocity);
    this->_target_velocity = velocity_;
}

auto
swerve_module_drive_motor::target_velocity() const
    -> units::velocity::meters_per_second_t {
    return this->_target_velocity;
}

auto
swerve_module_drive_motor::velocity() const
    -> units::velocity::meters_per_second_t {
    return units::velocity::meters_per_second_t {
        this->_encoder.GetVelocity()
    };
}

auto
swerve_module_drive_motor::stop() -> void {
    this->set_target_velocity(0.0_mps);
}

auto
swerve_module_drive_motor::controller() -> rev::CANSparkMax & {
    return _controller;
}

auto
swerve_module_drive_motor::encoder() -> rev::SparkRelativeEncoder & {
    return _encoder;
}

auto
swerve_module_drive_motor::pid_controller() -> rev::SparkPIDController & {
    return _pid_controller;
}

auto
swerve_module_drive_motor::log_values() -> void {
    frc::SmartDashboard::PutNumber(std::to_string(_controller.GetDeviceId())
                                       + " C-Velocity",
                                   velocity().value());
    frc::SmartDashboard::PutNumber(std::to_string(_controller.GetDeviceId())
                                       + " T-Velocity",
                                   target_velocity().value());
}

// Azimuth motors

swerve_module_azimuth_motor::swerve_module_azimuth_motor(
    util::motor_controller_settings motor_settings_,
    util::encoder_settings          encoder_settings_,
    util::pid_settings              pid_settings_)
    : _controller(motor_settings_.id, rev::CANSparkMax::MotorType::kBrushless)
    , _encoder(_controller.GetEncoder())
    , _pid_controller(_controller.GetPIDController())
    , _target_angle(0_rad)
    , _disabled(motor_settings_.is_disabled) {
    _controller.SetInverted(motor_settings_.is_inverted);
    _controller.SetOpenLoopRampRate(motor_settings_.open_ramp_rate.value());
    _controller.SetClosedLoopRampRate(motor_settings_.closed_ramp_rate.value());
    _controller.SetIdleMode(motor_settings_.idle_mode);

    _pid_controller.SetP(pid_settings_.k_p);
    _pid_controller.SetI(pid_settings_.k_i);
    _pid_controller.SetD(pid_settings_.k_d);
    _pid_controller.SetFF(pid_settings_.k_f);
    _pid_controller.SetOutputRange(-1.0, 1.0);

    _pid_controller.SetPositionPIDWrappingEnabled(true);
    _pid_controller.SetPositionPIDWrappingMaxInput(std::numbers::pi);
    _pid_controller.SetPositionPIDWrappingMinInput(-std::numbers::pi);

    _encoder.SetPositionConversionFactor(
        encoder_settings_.position_conversion_factor);
    _encoder.SetVelocityConversionFactor(
        encoder_settings_.velocity_conversion_factor);
}

auto
swerve_module_azimuth_motor::set_target_angle(units::angle::radian_t angle_)
    -> void {
    if (_disabled) {
        return;
    }

    _pid_controller.SetReference(angle_.value(),
                                 rev::CANSparkMax::ControlType::kPosition);
    this->_target_angle = angle_;
}

auto
swerve_module_azimuth_motor::seed_angle(units::angle::radian_t angle_) -> void {
    std::cout << "Seeded encoders\n";
    this->_encoder.SetPosition(angle_.value());
}

auto
swerve_module_azimuth_motor::target_angle() const -> units::angle::radian_t {
    return this->_target_angle;
}

auto
swerve_module_azimuth_motor::angle() const -> units::angle::radian_t {
    return units::angle::radian_t { _encoder.GetPosition() };
}

auto
swerve_module_azimuth_motor::controller() -> rev::CANSparkMax & {
    return _controller;
}

auto
swerve_module_azimuth_motor::encoder() -> rev::SparkRelativeEncoder & {
    return _encoder;
}

auto
swerve_module_azimuth_motor::pid_controller() -> rev::SparkPIDController & {
    return _pid_controller;
}

auto
swerve_module_azimuth_motor::log_values() -> void {
    frc::SmartDashboard::PutNumber(std::to_string(_controller.GetDeviceId())
                                       + " C-Angle",
                                   angle().value());

    frc::SmartDashboard::PutNumber(std::to_string(_controller.GetDeviceId())
                                       + " T-Angle",
                                   target_angle().value());
}

} // namespace td::drive
