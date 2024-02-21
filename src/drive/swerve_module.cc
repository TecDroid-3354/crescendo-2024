#include "swerve_module.hh"

#include "constants/drive.hh"
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>
#include <rev/CANSparkBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <string>
#include <units/angle.h>
#include <units/velocity.h>

namespace td {

swerve_module::swerve_module(swerve_module_config config,
                             frc::Translation2d   position)
    : _azimuth(config.azimuth_id, rev::CANSparkBase::MotorType::kBrushless)
    , _drive(config.drive_id, rev::CANSparkBase::MotorType::kBrushless)
    , _azimuth_encoder(
          _azimuth.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor))
    , _drive_encoder(
          _drive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor))
    , _azimuth_pid(_azimuth.GetPIDController())
    , _drive_pid(_drive.GetPIDController())
    , _cancoder(config.cancoder_id)
    , _offset(position) {
    _azimuth_pid.SetP(k::swerve::azimuth::K_P);
    _azimuth_pid.SetI(k::swerve::azimuth::K_I);
    _azimuth_pid.SetD(k::swerve::azimuth::K_D);
    _azimuth_pid.SetFF(k::swerve::azimuth::K_F);
    _azimuth_encoder.SetPositionConversionFactor(
        k::swerve::azimuth::position_conversion_factor);
    _azimuth_encoder.SetVelocityConversionFactor(
        k::swerve::azimuth::velocity_conversion_factor);

    _azimuth_pid.SetOutputRange(-1.0, 1.0);
    _azimuth.SetClosedLoopRampRate(k::swerve::azimuth::ramp_rate.value());

    _azimuth_pid.SetPositionPIDWrappingEnabled(true);
    _azimuth_pid.SetPositionPIDWrappingMinInput(-std::numbers::pi);
    _azimuth_pid.SetPositionPIDWrappingMaxInput(std::numbers::pi);

    _drive_pid.SetP(k::swerve::drive::K_P);
    _drive_pid.SetI(k::swerve::drive::K_I);
    _drive_pid.SetD(k::swerve::drive::K_D);
    _drive_pid.SetFF(k::swerve::drive::K_F);
    _drive_encoder.SetPositionConversionFactor(
        k::swerve::drive::position_conversion_factor);
    _drive_encoder.SetVelocityConversionFactor(
        k::swerve::drive::velocity_conversion_factor);

    _drive_pid.SetOutputRange(-1.0, 1.0);
    _drive.SetClosedLoopRampRate(k::swerve::drive::ramp_rate.value());

    _drive.SetInverted(config.drive_inverted);
}

auto
swerve_module::module_offset_from_center() -> frc::Translation2d {
    return _offset;
}

auto
swerve_module::optimize_state(frc::SwerveModuleState const &state)
    -> frc::SwerveModuleState {
    return frc::SwerveModuleState::Optimize(state, azimuth_angle());
}

auto
swerve_module::adopt_state(frc::SwerveModuleState state) -> void {
    _azimuth_pid.SetReference(state.angle.Radians().value(),
                              rev::CANSparkBase::ControlType::kPosition);

    _drive_pid.SetReference(state.speed.value(),
                            rev::CANSparkBase::ControlType::kVelocity);
}

auto
swerve_module::azimuth_angle() -> units::radian_t {
    return units::radian_t { _azimuth_encoder.GetPosition() };
}

auto
swerve_module::absolute_azimuth_angle() -> units::radian_t {
    return _cancoder.GetAbsolutePosition().GetValue();
}

auto
swerve_module::drive_velocity() -> units::meters_per_second_t {
    return units::meters_per_second_t { _drive_encoder.GetVelocity() };
}

auto
swerve_module::zero_azimuth() -> void {
    _azimuth_pid.SetReference(0, rev::CANSparkBase::ControlType::kPosition);
}

auto
swerve_module::sync_integrated_encoder_with_cancoder() -> void {
    _azimuth_encoder.SetPosition(absolute_azimuth_angle().value());
}

auto
swerve_module::full_reset() -> void {
    _cancoder.SetPosition(units::turn_t { 0.0 });
    _azimuth_encoder.SetPosition(0.0);
    _drive_encoder.SetPosition(0.0);
}

auto
swerve_module::reset_position() -> void {
    _azimuth_encoder.SetPosition(0.0);
    _drive_encoder.SetPosition(0.0);
}

auto
swerve_module::log() -> void {
    frc::SmartDashboard::PutNumber(std::to_string(_azimuth.GetDeviceId())
                                       + "Azimuth Angle",
                                   azimuth_angle().value());

    frc::SmartDashboard::PutNumber(std::to_string(_azimuth.GetDeviceId())
                                       + "Azimuth Angle AbsEnc",
                                   absolute_azimuth_angle().value());

    frc::SmartDashboard::PutNumber(std::to_string(_azimuth.GetDeviceId())
                                       + "Drive Velocity",
                                   drive_velocity().value());
}

} // namespace td
