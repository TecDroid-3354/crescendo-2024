#pragma once

#include "constants/util.hh"
#include "units/angle.h"
#include <rev/CANSparkMax.h>
#include <units/velocity.h>

namespace td::drive {

class swerve_module_drive_motor {
public:
    explicit swerve_module_drive_motor(
        util::motor_controller_settings motor_settings_,
        util::encoder_settings          encoder_settings_,
        util::pid_settings              pid_settings_);

    auto
    set_target_velocity(units::velocity::meters_per_second_t velocity_) -> void;

    [[nodiscard]] auto
    target_velocity() const -> units::velocity::meters_per_second_t;

    [[nodiscard]] auto
    velocity() const -> units::velocity::meters_per_second_t;

    [[nodiscard]] auto
    controller() -> rev::CANSparkMax &;

    [[nodiscard]] auto
    encoder() -> rev::SparkRelativeEncoder &;

    [[nodiscard]] auto
    pid_controller() -> rev::SparkPIDController &;

    auto
    stop() -> void;

    auto
    log_values() -> void;

private:
    rev::CANSparkMax                     _controller;
    rev::SparkRelativeEncoder            _encoder;
    rev::SparkPIDController              _pid_controller;
    units::velocity::meters_per_second_t _target_velocity;
    bool                                 _disabled = false;
};

class swerve_module_azimuth_motor {
public:
    explicit swerve_module_azimuth_motor(
        util::motor_controller_settings motor_settings_,
        util::encoder_settings          encoder_settings_,
        util::pid_settings              pid_settings_);

    auto
    set_target_angle(units::angle::radian_t angle_) -> void;

    auto
    seed_angle(units::angle::radian_t angle_) -> void;

    [[nodiscard]] auto
    target_angle() const -> units::angle::radian_t;

    [[nodiscard]] auto
    angle() const -> units::angle::radian_t;

    [[nodiscard]] auto
    controller() -> rev::CANSparkMax &;

    [[nodiscard]] auto
    encoder() -> rev::SparkRelativeEncoder &;

    [[nodiscard]] auto
    pid_controller() -> rev::SparkPIDController &;

    auto
    log_values() -> void;

private:
    rev::CANSparkMax          _controller;
    rev::SparkRelativeEncoder _encoder;
    rev::SparkPIDController   _pid_controller;
    units::angle::radian_t    _target_angle;
    bool                      _disabled = false;
};

} // namespace td::drive
