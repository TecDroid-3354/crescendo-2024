#include "robot_container.hh"

#include "constants/drive.hh"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace xfrc {

robot_container::robot_container()
    : drive(td::k::swerve::module::module_ids,
            { td::k::swerve::module::forwards_offset,
              td::k::swerve::module::sideways_offset })
    , controller(0) { }

auto
robot_container::robot_init() -> uint8_t {
    return 0;
}

auto
robot_container::robot_periodic() -> uint8_t {
    drive.sync_modules_with_cancoders();
    drive.log();
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
    return 0;
}

auto
robot_container::autonomous_periodic() -> uint8_t {
    return 0;
}

auto
robot_container::teleop_init() -> uint8_t {
    drive.SetDefaultCommand(frc2::cmd::Run(
        [this] {
            this->drive.drive(-controller.GetLeftY() * td::k::swerve::velocity,
                              +controller.GetLeftX() * td::k::swerve::velocity,
                              controller.GetRightX()
                                  * td::k::swerve::angular_velocity);
        },
        { &drive }));
    return 0;
}

auto
robot_container::teleop_periodic() -> uint8_t {
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
