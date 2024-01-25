#ifndef XFRC_CORE_ROBOT_CONTAINER_HH
#define XFRC_CORE_ROBOT_CONTAINER_HH

#include "constants/drive.hh"
#include "drive/swerve_drive.hh"
#include <cstdint>
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

private:
    td::swerve_drive drive {
        td::k::swerve::module::module_ids,
        {td::k::swerve::module::forwards_offset,
                                         td::k::swerve::module::sideways_offset}
    };

    frc2::CommandXboxController controller { 0 };
};

} // namespace xfrc

#endif
