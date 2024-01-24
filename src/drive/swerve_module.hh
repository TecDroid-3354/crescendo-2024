#pragma once

#include <cstdint>
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

struct swerve_module_ids {
    uint8_t azimuth_id;
    uint8_t drive_id;
    uint8_t cancoder_id;
};

class swerve_module {
public:
    explicit swerve_module(swerve_module_ids ids, frc::Translation2d position);

    [[nodiscard]] auto
    module_offset_from_center() -> frc::Translation2d;

    auto
    adopt_state(frc::SwerveModuleState state) -> void;

    auto
    optimize_and_adopt_state(frc::SwerveModuleState state) -> void;

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

    frc::AnalogEncoder _azimuth_encoder_abs;

    frc::Translation2d _offset;
};

} // namespace td
