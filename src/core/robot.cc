#include "robot.hh"

#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <stdexcept>
#include <string>

namespace xfrc {

robot::robot() = default;

auto
robot::RobotInit() -> void {
    std::cout << "lol\n";
    uint8_t status = _container.robot_init();
    _container.set_up_binds();

    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "robot_init returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::RobotPeriodic() -> void {
    frc2::CommandScheduler::GetInstance().Run();
    uint8_t status = _container.robot_periodic();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "robot_periodic returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::DisabledInit() -> void {
    uint8_t status = _container.disabled_init();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "disabled_init returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::DisabledPeriodic() -> void {
    uint8_t status = _container.disabled_periodic();
    if (status != 0) {
        throw std::runtime_error {
            std::string("disabled_periodic returned a status of ")
            + std::to_string(status) + ". Terminating"
        };
    }
}

auto
robot::AutonomousInit() -> void {
    uint8_t status = _container.autonomous_init();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "autonomous_init returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::AutonomousPeriodic() -> void {
    uint8_t status = _container.autonomous_periodic();
    if (status != 0) {
        throw std::runtime_error {
            std::string("autonomous_periodic returned a status of ")
            + std::to_string(status) + ". Terminating"
        };
    }
}

auto
robot::TeleopInit() -> void {
    uint8_t status = _container.teleop_init();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "teleop_init returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::TeleopPeriodic() -> void {
    uint8_t status = _container.teleop_periodic();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "teleop_periodic returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::TestPeriodic() -> void {
    uint8_t status = _container.test_periodic();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "test_periodic returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::SimulationInit() -> void {
    uint8_t status = _container.simulation_init();
    if (status != 0) {
        throw std::runtime_error { std::string(
                                       "simulaton_init returned a status of ")
                                   + std::to_string(status) + ". Terminating" };
    }
}

auto
robot::SimulationPeriodic() -> void {
    uint8_t status = _container.simulation_init();
    if (status != 0) {
        throw std::runtime_error {
            std::string("simulation_periodic returned a status of ")
            + std::to_string(status) + ". Terminating"
        };
    }
}

} // namespace xfrc

//
