#include "Robot.h"
#include <fmt/core.h>

void Robot::RobotInit()
{
    // swerve motor config
    driveMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    driveMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    driveMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
    driveMotor4.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
}

void Robot::RobotPeriodic(){}

void Robot::AutonomousInit()
{
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);
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
