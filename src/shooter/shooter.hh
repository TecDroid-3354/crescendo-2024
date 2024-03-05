#pragma once

#include "constants/shooter.hh"
#include "constants/util.hh"
#include "rev/CANSparkMax.h"
#include "rev/SparkRelativeEncoder.h"
#include "units/velocity.h"

namespace td::subsystem::shooter {

class shooter {
public:
    explicit shooter(k::shooter_settings settings_);

    auto
    set_velocity(units::velocity::meters_per_second_t velocity_) -> void;

    auto
    set_percentage(double percentage_) -> void;

    auto
    stop() -> void;

private:
    rev::CANSparkMax _left_motor;
    rev::CANSparkMax _right_motor;

    rev::SparkRelativeEncoder _left_encoder;
    rev::SparkRelativeEncoder _right_encoder;

    rev::SparkPIDController _left_pid_controller;
    rev::SparkPIDController _right_pid_controller;
};

} // namespace td::subsystem::shooter
