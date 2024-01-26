#pragma once

#include "units/velocity.h"
#include <cstdint>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/AnalogEncoder.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/angle.h>
#include <units/length.h>

namespace td {

struct swerve_module_config {
    uint8_t azimuth_id;
    uint8_t drive_id;
    uint8_t cancoder_id;
    bool    drive_inverted;
};

class swerve_module {
public:
    explicit swerve_module(swerve_module_config config,
                           frc::Translation2d   position);

    [[nodiscard]] auto
    module_offset_from_center() -> frc::Translation2d;

    auto
    adopt_state(frc::SwerveModuleState state) -> void;

    auto
    optimize_and_adopt_state(frc::SwerveModuleState state) -> void;

    auto
    azimuth_angle() -> units::radian_t;

    auto
    drive_velocity() -> units::meters_per_second_t;

    auto
    align_forwards() -> void;

    auto
    reset() -> void;

    auto
    log() -> void;

private:
    rev::CANSparkMax _azimuth;
    rev::CANSparkMax _drive;

    rev::SparkRelativeEncoder _azimuth_encoder;
    rev::SparkRelativeEncoder _drive_encoder;

    rev::SparkPIDController _azimuth_pid;
    rev::SparkPIDController _drive_pid;

    ctre::phoenix6::hardware::CANcoder _cancoder;

    frc::Translation2d _offset;
};

} // namespace td
