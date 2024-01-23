#include "swerve_drive.hh"

#include "frc/kinematics/ChassisSpeeds.h"
#include "units/velocity.h"

namespace td {

swerve_drive::swerve_drive(std::array<swerve_module_ids, 4> module_ids,
                           units::meter_t                   offset)
    : _front_right(module_ids [0], { offset, -offset })
    , _front_left(module_ids [1], { offset, offset })
    , _back_left(module_ids [2], { -offset, offset })
    , _back_right(module_ids [3], { -offset, -offset })
    , _kinematics(_front_right.module_offset_from_center(),
                  _front_left.module_offset_from_center(),
                  _back_left.module_offset_from_center(),
                  _back_right.module_offset_from_center()) { }

auto
swerve_drive::get_states_for(units::meters_per_second_t  x_vel,
                             units::meters_per_second_t  y_vel,
                             units::radians_per_second_t angular_velocity)
    -> std::array<frc::SwerveModuleState, 4> {
    frc::ChassisSpeeds speeds = { x_vel, -y_vel, angular_velocity };

    return _kinematics.ToSwerveModuleStates(speeds);
}

auto
swerve_drive::adopt_states(std::array<frc::SwerveModuleState, 4> states)
    -> void {
    _front_right.adopt_state(states [0]);
    _front_left.adopt_state(states [1]);
    _back_left.adopt_state(states [2]);
    _back_right.adopt_state(states [3]);
}

auto
swerve_drive::optimize_and_adopt_states(
    std::array<frc::SwerveModuleState, 4> states) -> void {
    _front_right.optimize_and_adopt_state(states [0]);
    _front_left.optimize_and_adopt_state(states [1]);
    _back_left.optimize_and_adopt_state(states [2]);
    _back_right.optimize_and_adopt_state(states [3]);
}

auto
swerve_drive::drive(units::meters_per_second_t  x_vel,
                    units::meters_per_second_t  y_vel,
                    units::radians_per_second_t angular_velocity) -> void {
    auto states = get_states_for(x_vel, y_vel, angular_velocity);
    // optimize_and_adopt_states(states);
}

auto
swerve_drive::field_oriented_drive(units::meters_per_second_t  x_vel,
                                   units::meters_per_second_t  y_vel,
                                   units::radians_per_second_t angular_velocity)
    -> void {
    // TODO: MAKE IT FIELD ORIENTED
    optimize_and_adopt_states(get_states_for(x_vel, y_vel, angular_velocity));
}

auto
swerve_drive::log() -> void {
    _front_right.log();
    _front_left.log();
    _back_left.log();
    _back_right.log();
}

} // namespace td
