#pragma once

#include "constants/util.hh"
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>

namespace td {
struct swerve_drive_motor_settings {
    util::motor_controller_settings drive_settings;
    util::motor_controller_settings azimuth_settings;
};

struct swerve_encoder_settings {
    util::encoder_settings drive_settings;
    util::encoder_settings azimuth_settings;
};

struct swerve_pid_settings {
    util::pid_settings drive_settings;
    util::pid_settings azimuth_settings;
};

struct swerve_module_settings {
    uint8_t                     absolute_encoder_id = 0;
    swerve_drive_motor_settings motor_settings;
    swerve_encoder_settings     encoder_settings;
    swerve_pid_settings         pid_settings;
    frc::Translation2d          manhattan_offset_from_robot_center;
};

struct swerve_drive_settings {
    swerve_module_settings front_right_settings;
    swerve_module_settings front_left_settings;
    swerve_module_settings back_left_settings;
    swerve_module_settings back_right_settings;
};

struct swerve_drive_target_states {
    frc::SwerveModuleState front_right_target;
    frc::SwerveModuleState front_left_target;
    frc::SwerveModuleState back_left_target;
    frc::SwerveModuleState back_right_target;
};

} // namespace td
