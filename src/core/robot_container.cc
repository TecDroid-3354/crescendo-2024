#include "robot_container.hh"

#include "constants/drive.hh"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace xfrc {

robot_container::robot_container() { }

auto
robot_container::robot_init() -> uint8_t {
    return 0;
}

auto
robot_container::robot_periodic() -> uint8_t {
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
    return 0;
}

auto
robot_container::teleop_periodic() -> uint8_t {
    drive.drive(td::k::swerve::drive::velocity * controller.GetLeftX(),
                td::k::swerve::drive::velocity * controller.GetLeftY(),
                td::k::swerve::azimuth::angular_velocity
                    * controller.GetRightX());

    frc::SmartDashboard::PutNumber("Controller LX", controller.GetLeftX());
    frc::SmartDashboard::PutNumber("Controller LY", controller.GetLeftY());
    frc::SmartDashboard::PutNumber("Controller RX", controller.GetRightX());

    drive.log();

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

} // namespace xfrc
