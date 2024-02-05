#pragma once

#include <cstdint>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/AnalogEncoder.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/MotorFeedbackSensor.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

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

    /**
     * Returns how far away this module is from the center
     */
    [[nodiscard]] auto
    module_offset_from_center() -> frc::Translation2d;

    auto
    optimize_state(frc::SwerveModuleState const &state)
        -> frc::SwerveModuleState;

    /**
     * Causes this module to physically adopt the given state
     */
    auto
    adopt_state(frc::SwerveModuleState state) -> void;

    /**
     * Syncs with the absolute encoder and drives the azimuth to angle 0
     */
    auto
    zero_azimuth() -> void;

    /**
     * Sets the NEO's integrated encoder to be in sync with the absolute encoder
     */
    auto
    sync_integrated_encoder_with_cancoder() -> void;

    /**
     * Returns the azimuth's angle
     */
    auto
    azimuth_angle() -> units::radian_t;

    /**
     * Returns the azimuth's angle (reported from absolute encoder)
     */
    auto
    absolute_azimuth_angle() -> units::radian_t;

    /**
     * Returns the drive's velocity
     */
    auto
    drive_velocity() -> units::meters_per_second_t;

    /**
     * Resets both motos' encoders and the absolute encoder
     */
    auto
    full_reset() -> void;

    /**
     * Resets both motos' encoders but not the absolute encoder
     */
    auto
    reset_position() -> void;

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
