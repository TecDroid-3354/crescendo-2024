#include "robot_container.hh"

#include "constants/drive.hh"
#include "constants/indexer.hh"
#include "constants/shooter.hh"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace xfrc {

robot_container::robot_container()
    : drive(td::drive::k::SWERVE_SETTINGS)
    , shooter(td::subsystem::shooter::k::SHOOTER_SETTINGS)
    , indexer(td::subsystem::indexer::k::INDEXER_SETTINGS)
    , controller(0) { }

auto
robot_container::robot_init() -> uint8_t {
    drive.seed_azimuth_encoders();

    return 0;
}

auto
robot_container::robot_periodic() -> uint8_t {
    drive.log_values();
    frc::SmartDashboard::PutNumber("Controller LX", controller.GetLeftX());
    frc::SmartDashboard::PutNumber("Controller LY", controller.GetLeftY());
    frc::SmartDashboard::PutNumber("Controller RX", controller.GetRightX());
    return 0;
}

auto
robot_container::disabled_init() -> uint8_t {
    return 0;
}

auto
robot_container::disabled_periodic() -> uint8_t {
    return 0;
}

auto
robot_container::autonomous_init() -> uint8_t {
    drive.seed_azimuth_encoders();

    return 0;
}

auto
robot_container::autonomous_periodic() -> uint8_t {
    return 0;
}

auto
robot_container::teleop_init() -> uint8_t {
    drive.seed_azimuth_encoders();

    drive.SetDefaultCommand(frc2::cmd::Run(
        [this] {
            if (controller.GetXButton()) {
                drive.seed_azimuth_encoders();
                drive.set_module_angles(0_rad);
            } else {
                this->drive.drive(
                    controller.GetLeftY() * td::drive::k::MAX_VELOCITY,
                    controller.GetLeftX() * td::drive::k::MAX_VELOCITY,
                    controller.GetRightX() * td::drive::k::MAX_ANGULAR_VELOCITY,
                    td::drive::swerve_drive_orientation_target::ROBOT_CENTRIC);
            }
        },
        { &drive }));

    return 0;
}

auto
robot_container::teleop_periodic() -> uint8_t {
    shooter.set_percentage(controller.GetRightTriggerAxis()
                           - controller.GetLeftTriggerAxis());

    indexer.set_percentage(controller.GetBButton() ? 0.8 : 0);
    return 0;
}

auto
robot_container::test_periodic() -> uint8_t {
    return 0;
}

auto
robot_container::simulation_init() -> uint8_t {
    return 0;
}

auto
robot_container::simulation_periodic() -> uint8_t {
    return 0;
}

auto
robot_container::set_up_binds() -> void { }

} // namespace xfrc
