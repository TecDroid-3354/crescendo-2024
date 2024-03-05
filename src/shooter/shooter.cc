#include "shooter.hh"

namespace td::subsystem::shooter {

shooter::shooter(k::shooter_settings settings_)
    : _left_motor(settings_.left_controller_settings.id,
                  settings_.left_controller_settings.motor_type)
    , _right_motor(settings_.right_controller_settings.id,
                   settings_.right_controller_settings.motor_type)
    , _left_encoder(_left_motor.GetEncoder())
    , _right_encoder(_right_motor.GetEncoder())
    , _left_pid_controller(_left_motor.GetPIDController())
    , _right_pid_controller(_right_motor.GetPIDController()) {
    _left_motor.SetInverted(settings_.left_controller_settings.is_inverted);
    _right_motor.SetInverted(settings_.right_controller_settings.is_inverted);
    _left_motor.SetIdleMode(settings_.left_controller_settings.idle_mode);
    _right_motor.SetIdleMode(settings_.right_controller_settings.idle_mode);
}

auto
shooter::set_velocity(units::velocity::meters_per_second_t velocity_) -> void {
    _left_pid_controller.SetReference(velocity_.value(),
                                      rev::CANSparkMax::ControlType::kVelocity);
    _right_pid_controller.SetReference(
        velocity_.value(),
        rev::CANSparkMax::ControlType::kVelocity);
}

auto
shooter::set_percentage(double percentage_) -> void {
    _left_motor.Set(percentage_);
    _right_motor.Set(percentage_);
}

auto
shooter::stop() -> void {
    set_percentage(0);
}

} // namespace td::subsystem::shooter
