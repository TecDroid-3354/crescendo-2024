#pragma once

#include "constants/intake.hh"
#include "rev/CANSparkMax.h"
#include "units/velocity.h"

namespace td::subsystem::intake {

class intake {
public:
    explicit intake(k::intake_settings settings_);

    auto
    set_velocity(units::velocity::meters_per_second_t velocity_) -> void;

    auto
    set_percentage(double percentage_) -> void;

    auto
    stop() -> void;

private:
    rev::CANSparkMax          _controller;
    rev::SparkRelativeEncoder _encoder;
    rev::SparkPIDController   _pid_controller;
};
} // namespace td::subsystem::intake
