#ifndef XFRC_CORE_ROBOT_CONTAINER_HH
#define XFRC_CORE_ROBOT_CONTAINER_HH

#include <cstdint>

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
};

} // namespace xfrc

#endif
