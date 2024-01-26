#include "robot_container.hh"

#include "constants/drive.hh"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include <algorithm>
#include <utility>

namespace xfrc {

robot_container::robot_container()
    : drive(td::k::swerve::module::module_ids,
            { td::k::swerve::module::forwards_offset,
              td::k::swerve::module::sideways_offset })
    , controller(0)

    , robot_oriented_drive_cmd(frc2::cmd::Run(
          [this] {
              this->drive.drive_optimized(
                  -controller.GetLeftY() * td::k::swerve::drive::velocity,
                  controller.GetLeftX() * td::k::swerve::drive::velocity,
                  controller.GetRightX()
                      * td::k::swerve::azimuth::angular_velocity);
          },
          { &drive }))

    , field_oriented_drive_cmd(frc2::cmd::Run(
          [this] {
              this->drive.field_oriented_drive_optimized(
                  -controller.GetLeftY() * td::k::swerve::drive::velocity,
                  controller.GetLeftX() * td::k::swerve::drive::velocity,
                  controller.GetRightX()
                      * td::k::swerve::azimuth::angular_velocity);
          },
          { &drive }))

    , align_swerve_cmd(frc2::cmd::RunOnce(
          [this] {
              this->drive.align_modules_forwards();
          },
          { &drive }))

    , reset_swerve_cmd(frc2::cmd::RunOnce(
          [this] {
              this->drive.reset();
          },
          { &drive })) { }

auto
robot_container::robot_init() -> uint8_t {
    drive.SetDefaultCommand(std::forward<decltype(field_oriented_drive_cmd)>(
        robot_oriented_drive_cmd));
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

auto
robot_container::set_up_binds() -> void {
    (controller.Start() && controller.A())
        .OnTrue(std::forward<decltype(reset_swerve_cmd)>(reset_swerve_cmd));

    // controller.A().WhileTrue(
    //     std::forward<decltype(align_swerve_cmd)>(align_swerve_cmd));
    controller.X().WhileTrue(std::move(align_swerve_cmd));
}

} // namespace xfrc
