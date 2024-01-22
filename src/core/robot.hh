#pragma once

#include "robot_container.hh"
#include <frc/TimedRobot.h>

namespace xfrc {

class robot : public frc::TimedRobot {
public:
    explicit robot();

    auto
    RobotInit() -> void override;

    auto
    RobotPeriodic() -> void override;

    auto
    DisabledInit() -> void override;

    auto
    DisabledPeriodic() -> void override;

    auto
    AutonomousInit() -> void override;

    auto
    AutonomousPeriodic() -> void override;

    auto
    TeleopInit() -> void override;

    auto
    TeleopPeriodic() -> void override;

    auto
    TestPeriodic() -> void override;

    auto
    SimulationInit() -> void override;

    auto
    SimulationPeriodic() -> void override;

private:
    robot_container _container;
};

} // namespace xfrc
