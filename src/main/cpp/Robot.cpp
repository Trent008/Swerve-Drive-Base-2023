#include "Robot.h"

void Robot::RobotInit()
{
    swerve.initialize();
}

void Robot::RobotPeriodic(){}

void Robot::AutonomousInit(){}

void Robot::AutonomousPeriodic()
{
  // move the swerve drive twards the next setpoint
  swerveController.setReferencePose(setpoints[i].swerveSetpoint);
  // go to next setpoint if this setpoint has been reached
  if (swerveController.poseReached(3, 5) && (i < 14))
  {
    i++;
  }
}

void Robot::TeleopInit()
{}

void Robot::TeleopPeriodic()
{
  swerve.Set(xboxC.getFieldVelocity());

  if (xboxC.abxyAllTrue()) {
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
