#ifndef XFRC_CORE_ROBOT_CONTAINER_HH
#define XFRC_CORE_ROBOT_CONTAINER_HH

#include "constants/drive.hh"
#include "drive/swerve_drive.hh"
#include <cstdint>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>

namespace xfrc {

class robot_container {
public:
    explicit robot_container();

    auto
    robot_init() -> uint8_t;

    auto
    robot_periodic() -> uint8_t;

    auto
    disabled_init() -> uint8_t;

    auto
    disabled_periodic() -> uint8_t;

    auto
    autonomous_init() -> uint8_t;

    auto
    autonomous_periodic() -> uint8_t;

    auto
    teleop_init() -> uint8_t;

    auto
    teleop_periodic() -> uint8_t;

    auto
    test_periodic() -> uint8_t;

    auto
    simulation_init() -> uint8_t;

    auto
    simulation_periodic() -> uint8_t;

    auto
    set_up_binds() -> void;

private:
    td::swerve_drive            drive;
    frc2::CommandXboxController controller;
};

} // namespace xfrc

#endif
