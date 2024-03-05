#include "indexer.hh"

namespace td::subsystem::indexer {

indexer::indexer(k::indexer_settings settings_)
    : _controller(settings_.controller_settings.id,
                  settings_.controller_settings.motor_type)
    , _encoder(_controller.GetEncoder())
    , _pid_controller(_controller.GetPIDController()) {
    _controller.SetInverted(settings_.controller_settings.is_inverted),
        _controller.SetIdleMode(settings_.controller_settings.idle_mode),
        _controller.SetInverted(settings_.controller_settings.is_inverted),

        _encoder.SetPositionConversionFactor(
            settings_.encoder_settings.position_conversion_factor);
    _encoder.SetVelocityConversionFactor(
        settings_.encoder_settings.velocity_conversion_factor);

    _pid_controller.SetP(settings_.pid_settings.k_p);
    _pid_controller.SetI(settings_.pid_settings.k_i);
    _pid_controller.SetD(settings_.pid_settings.k_d);
    _pid_controller.SetFF(settings_.pid_settings.k_f);
}

auto
indexer::set_velocity(units::velocity::meters_per_second_t velocity_) -> void {
    _pid_controller.SetReference(velocity_.value(),
                                 rev::CANSparkMax::ControlType::kVelocity);
}

auto
indexer::set_percentage(double percentage_) -> void {
    _controller.Set(percentage_);
}

auto
indexer::stop() -> void {
    set_percentage(0);
}

} // namespace td::subsystem::indexer
