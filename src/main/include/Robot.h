#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "TeleopControllers/XBOXController.h"
#include "Swerve/SwervePoseController.h"
#include "AutonomousPreset.h"

class Robot : public frc::TimedRobot
{

public:
  int i = 0; // keeps track of the autonomous point index

  AutonomousSetpoint setpoints[15] =
        {
            {SwervePreset{Pose{{0, 0},0}, 0.3}},
            {SwervePreset{Pose{{0, 20},0}, 0.3}},
            {SwervePreset{Pose{{0, 0},0}, 0.3}},
            {},
            {},
            {},
            {},
            {},
            {},
            {},
            {},
            {},
            {},
            {},
            {}};
  
  XBOXController xboxC{new frc::Joystick{0}};

  // swerve drive object to control the 4-SwerveModule array
  SwervePoseController swerveController;

  
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
};
