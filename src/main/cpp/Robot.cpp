#include "Robot.h"
#include <fmt/core.h>

void Robot::RobotInit()
{
    // swerve motor config
    
}

void Robot::RobotPeriodic(){}

void Robot::AutonomousInit()
{
  configs::TalonFXConfiguration configs{};
  configs.Slot0.kP = 40; // An error of 1 rotations results in 40 amps output
  configs.Slot0.kD = 2; // A change of 1 rotation per second results in 2 amps output

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;  // Peak output of 130 amps
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -130; // Peak output of 130 amps
  drive_m1.GetConfigurator().Apply(configs);
  drive_m1.SetRotorPosition(0_tr);
  drive_m2.GetConfigurator().Apply(configs);
  drive_m2.SetRotorPosition(0_tr);
  drive_m3.GetConfigurator().Apply(configs);
  drive_m3.SetRotorPosition(0_tr);
  drive_m4.GetConfigurator().Apply(configs);
  drive_m4.SetRotorPosition(0_tr);

  swerve.zeroYaw();
}

void Robot::AutonomousPeriodic()
{
  // move the swerve drive twards the next setpoint
  swerveTargeting.targetPose(moduleArray, params.setpoints[i].swerveSetpoint);
  // go to next setpoint if this setpoint has been reached
  if (swerveTargeting.poseReached(3, 5) && (i < 14))
  {
    i++;
  }
}

void Robot::TeleopInit()
{}

void Robot::TeleopPeriodic()
{
  swerve.Set(moduleArray, xboxC.getFieldVelocity());

  if (xboxC.zero()) {
    swerve.zeroYaw();
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
