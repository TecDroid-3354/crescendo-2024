#include "swerve_drive.hh"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/math.h>
#include <units/velocity.h>

namespace td {

swerve_drive::swerve_drive(std::array<swerve_module_config, 4> module_ids,
                           frc::Translation2d                  offset)
    : _front_right(module_ids [0], { offset.X(), offset.Y() })
    , _front_left(module_ids [1], { offset.X(), -offset.Y() })
    , _back_left(module_ids [2], { -offset.X(), -offset.Y() })
    , _back_right(module_ids [3], { -offset.X(), offset.Y() })
    , _gyro(frc::SerialPort::kMXP)

    , _kinematics(_front_right.module_offset_from_center(),
                  _front_left.module_offset_from_center(),
                  _back_left.module_offset_from_center(),
                  _back_right.module_offset_from_center()) {
    _gyro.Reset();
    _gyro.ZeroYaw();
    _gyro.Calibrate();
}

auto
swerve_drive::get_robot_centric_states_for(
    units::meters_per_second_t  x_vel,
    units::meters_per_second_t  y_vel,
    units::radians_per_second_t angular_velocity)
    -> std::array<frc::SwerveModuleState, 4> {
    frc::ChassisSpeeds speeds = { x_vel, y_vel, angular_velocity };

    return _kinematics.ToSwerveModuleStates(speeds);
}

auto
swerve_drive::get_field_centric_states_for(
    units::meters_per_second_t  x_vel,
    units::meters_per_second_t  y_vel,
    units::radians_per_second_t angular_velocity)
    -> std::array<frc::SwerveModuleState, 4> {
    frc::ChassisSpeeds speeds =
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(x_vel,
                                                    y_vel,
                                                    angular_velocity,
                                                    heading());

    return _kinematics.ToSwerveModuleStates(speeds);
}

auto
swerve_drive::adopt_state_array(std::array<frc::SwerveModuleState, 4> states)
    -> void {
    _front_right.adopt_state(_front_right.optimize_state(states [0]));
    _front_left.adopt_state(_front_left.optimize_state(states [1]));
    _back_left.adopt_state(_back_left.optimize_state(states [2]));
    _back_right.adopt_state(_back_right.optimize_state(states [3]));
}

auto
swerve_drive::drive(units::meters_per_second_t  x_vel,
                    units::meters_per_second_t  y_vel,
                    units::radians_per_second_t angular_velocity) -> void {
    auto module_states =
        get_robot_centric_states_for(x_vel, y_vel, angular_velocity);
    adopt_state_array(module_states);
}

auto
swerve_drive::field_oriented_drive(units::meters_per_second_t  x_vel,
                                   units::meters_per_second_t  y_vel,
                                   units::radians_per_second_t angular_velocity)
    -> void {
    auto module_states =
        get_field_centric_states_for(x_vel, y_vel, angular_velocity);
    adopt_state_array(module_states);
}

auto
swerve_drive::sync_modules_with_cancoders() -> void {
    _front_right.sync_integrated_encoder_with_cancoder();
    _front_left.sync_integrated_encoder_with_cancoder();
    _back_left.sync_integrated_encoder_with_cancoder();
    _back_right.sync_integrated_encoder_with_cancoder();
}

auto
swerve_drive::zero_modules() -> void {
    _front_right.zero_azimuth();
    _front_left.zero_azimuth();
    _back_left.zero_azimuth();
    _back_right.zero_azimuth();
}

auto
swerve_drive::heading() -> units::radian_t {
    return units::degree_t { static_cast<double>(_gyro.GetYaw()) };
}

auto
swerve_drive::full_reset() -> void {
    _front_right.full_reset();
    _front_left.full_reset();
    _back_left.full_reset();
    _back_right.full_reset();
    _gyro.ZeroYaw();
}

auto
swerve_drive::reset() -> void {
    _front_right.reset_position();
    _front_left.reset_position();
    _back_left.reset_position();
    _back_right.reset_position();
    _gyro.ZeroYaw();
}

auto
swerve_drive::log() -> void {
    _front_right.log();
    _front_left.log();
    _back_left.log();
    _back_right.log();
    frc::SmartDashboard::PutNumber("Robot Angle", heading().value());
}

} // namespace td
