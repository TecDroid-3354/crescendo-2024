#include "swerve_module.hh"

#include "constants/drive.hh"
#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/CANSparkBase.h"
#include "rev/SparkAbsoluteEncoder.h"
#include "units/angle.h"
#include <string>

namespace td {

swerve_module::swerve_module(swerve_module_ids ids, frc::Translation2d position)
    : _azimuth(ids.azimuth_id, rev::CANSparkBase::MotorType::kBrushless)
    , _drive(ids.drive_id, rev::CANSparkBase::MotorType::kBrushless)
    , _azimuth_encoder(
          _azimuth.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor))
    , _drive_encoder(
          _drive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor))
    , _azimuth_pid(_azimuth.GetPIDController())
    , _drive_pid(_drive.GetPIDController())
    , _azimuth_encoder_abs(ids.cancoder_id)
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

    _drive_pid.SetP(k::swerve::drive::K_P);
    _drive_pid.SetI(k::swerve::drive::K_I);
    _drive_pid.SetD(k::swerve::drive::K_D);
    _drive_pid.SetFF(k::swerve::drive::K_F);
    _drive_encoder.SetPositionConversionFactor(
        k::swerve::drive::position_conversion_factor);
    _drive_encoder.SetVelocityConversionFactor(
        k::swerve::drive::velocity_conversion_factor);

    _drive_pid.SetOutputRange(-1.0, 1.0);
}

auto
swerve_module::module_offset_from_center() -> frc::Translation2d {
    return _offset;
}

auto
swerve_module::adopt_state(frc::SwerveModuleState state) -> void {
    // _drive_pid.SetReference(state.speed.value(),
    //                         rev::CANSparkBase::ControlType::kVelocity);

    _azimuth_pid.SetReference(state.angle.Radians().value(),
                              rev::CANSparkBase::ControlType::kPosition);
}

auto
swerve_module::optimize_and_adopt_state(frc::SwerveModuleState state) -> void {
    frc::SwerveModuleState new_state = frc::SwerveModuleState::Optimize(
        state,
        frc::Rotation2d { units::radian_t { _azimuth_encoder.GetPosition() } });

    // _drive_pid.SetReference(state.speed.value(),
    //                         rev::CANSparkBase::ControlType::kVelocity);

    _azimuth_pid.SetReference(new_state.angle.Radians().value(),
                              rev::CANSparkBase::ControlType::kPosition);

    frc::SmartDashboard::PutNumber(std::to_string(_azimuth.GetDeviceId())
                                       + "Target Angle",
                                   new_state.angle.Radians().value());
}

auto
swerve_module::log() -> void {
    frc::SmartDashboard::PutNumber(std::to_string(_azimuth.GetDeviceId())
                                       + "Azimuth Angle",
                                   _azimuth_encoder.GetPosition());

    frc::SmartDashboard::PutNumber(std::to_string(_azimuth.GetDeviceId())
                                       + "Drive Velocity",
                                   _drive_encoder.GetVelocity());

    _azimuth.Set(0.1);
}

} // namespace td
