#include "robot.hh"

#include <frc2/command/CommandScheduler.h>
#include <iostream>

namespace xfrc {

robot::robot() = default;

auto
robot::RobotInit() -> void {
    std::cout << "lol\n";
    _container.robot_init();
}

auto
robot::RobotPeriodic() -> void {
    frc2::CommandScheduler::GetInstance().Run();
    _container.robot_periodic();
}

auto
robot::DisabledInit() -> void {
    _container.disabled_init();
}

auto
robot::DisabledPeriodic() -> void {
    _container.disabled_periodic();
}

auto
robot::AutonomousInit() -> void {
    _container.autonomous_init();
}

auto
robot::AutonomousPeriodic() -> void {
    _container.autonomous_periodic();
}

auto
robot::TeleopInit() -> void {
    _container.autonomous_init();
}

auto
robot::TeleopPeriodic() -> void {
    _container.teleop_periodic();
}

auto
robot::TestPeriodic() -> void {
    _container.test_periodic();
}

auto
robot::SimulationInit() -> void {
    _container.simulation_init();
}

auto
robot::SimulationPeriodic() -> void {
    _container.simulation_init();
}

} // namespace xfrc

//
