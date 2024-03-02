#include "swerve_drive.hh"

#include "frc/kinematics/SwerveModuleState.h"
#include <array>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <iostream>
#include <units/angle.h>
#include <units/math.h>
#include <units/velocity.h>

namespace td::drive {

swerve_drive::swerve_drive(swerve_drive_settings settings_)
    : _front_right(settings_.front_right_settings)
    , _front_left(settings_.front_left_settings)
    , _back_left(settings_.back_left_settings)
    , _back_right(settings_.back_right_settings)
    , _gyro(frc::SerialPort::kMXP)
    , _kinematics(
          settings_.front_right_settings.manhattan_offset_from_robot_center,
          settings_.front_left_settings.manhattan_offset_from_robot_center,
          settings_.back_left_settings.manhattan_offset_from_robot_center,
          settings_.back_right_settings.manhattan_offset_from_robot_center) { }

auto
swerve_drive::calculate_target_states(
    units::velocity::meters_per_second_t          forward_velocity_,
    units::velocity::meters_per_second_t          sideways_velocity_,
    units::angular_velocity::radians_per_second_t angular_velocity_,
    swerve_drive_orientation_target               target_,
    frc::Translation2d center_of_rotation_) -> swerve_drive_target_states {
    std::array<frc::SwerveModuleState, 4> states;
    switch (target_) {
    case ROBOT_CENTRIC :
        states = _kinematics.ToSwerveModuleStates(
            frc::ChassisSpeeds { forward_velocity_,
                                 sideways_velocity_,
                                 angular_velocity_ },
            center_of_rotation_);
        break;
    case FIELD_CENTRIC :
        states = _kinematics.ToSwerveModuleStates(
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                forward_velocity_,
                sideways_velocity_,
                angular_velocity_,
                { units::angle::radian_t {
                    static_cast<double>(_gyro.GetYaw()) } }),
            center_of_rotation_);
        break;
    case GLOBE_CENTRIC :
        states = _kinematics.ToSwerveModuleStates(
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                forward_velocity_,
                sideways_velocity_,
                angular_velocity_,
                { units::angle::radian_t {
                    static_cast<double>(_gyro.GetCompassHeading()) } }),
            center_of_rotation_);
        break;
    }

    swerve_drive_target_states resulting_states {
        .front_right_target = _front_right.optimize_state(states [FRM_IDX]),
        .front_left_target  = _front_left.optimize_state(states [FLM_IDX]),
        .back_left_target   = _back_left.optimize_state(states [BLM_IDX]),
        .back_right_target  = _back_right.optimize_state(states [BRM_IDX])
    };

    return resulting_states;
}

auto
swerve_drive::drive(
    units::velocity::meters_per_second_t          forward_velocity_,
    units::velocity::meters_per_second_t          sideways_velocity_,
    units::angular_velocity::radians_per_second_t angular_velocity_,
    swerve_drive_orientation_target               target_orientation_,
    frc::Translation2d                            center_of_rotation_) -> void {
    swerve_drive_target_states states =
        calculate_target_states(forward_velocity_,
                                sideways_velocity_,
                                angular_velocity_,
                                target_orientation_,
                                center_of_rotation_);

    _front_right.adopt_state(states.front_right_target);
    _front_left.adopt_state(states.front_left_target);
    _back_left.adopt_state(states.back_left_target);
    _back_right.adopt_state(states.back_right_target);
}

auto
swerve_drive::seed_azimuth_encoders() -> void {
    _front_right.seed_angle_from_absolute_encoder();
    _front_left.seed_angle_from_absolute_encoder();
    _back_left.seed_angle_from_absolute_encoder();
    _back_right.seed_angle_from_absolute_encoder();
}

auto
swerve_drive::set_module_angles(units::angle::radian_t angle_) -> void {
    _front_right.azimuth_motor().set_target_angle(angle_);
    _front_left.azimuth_motor().set_target_angle(angle_);
    _back_left.azimuth_motor().set_target_angle(angle_);
    _back_right.azimuth_motor().set_target_angle(angle_);
}

auto
swerve_drive::log_values() -> void {
    _front_right.log_values();
    _front_left.log_values();
    _back_left.log_values();
    _back_right.log_values();
}

} // namespace td::drive
