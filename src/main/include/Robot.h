#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "XBOXController.h"
#include "SwervePoseTargeting.h"

class Robot : public frc::TimedRobot
{

public:
  int i = 0; // keeps track of the autonomous point index
  
  frc::Joystick driveController{0}; //
  XBOXController xboxC{&driveController};


  /* -------- swerve drive motors -------- */
  WPI_TalonFX driveMotor1{11};
  WPI_TalonFX driveMotor2{12};
  WPI_TalonFX driveMotor3{13};
  WPI_TalonFX driveMotor4{14};
  /* -------- swerve module encoders -------- */
  CANCoder encoder1{21};
  CANCoder encoder2{22};
  CANCoder encoder3{23};
  CANCoder encoder4{24};
  /* -------- swerve module wheel turning motors -------- */
  rev::CANSparkMax steeringMotor1{31, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor2{32, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor3{33, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor4{34, rev::CANSparkMax::MotorType::kBrushless};

  // swerve module array:
  SwerveModule moduleArray[4] =
  {
    {&driveMotor1, &steeringMotor1, &encoder1, {-17.75, 25}},
    {&driveMotor2, &steeringMotor2, &encoder2, {-17.75, -25}},
    {&driveMotor3, &steeringMotor3, &encoder3, {17.75, 25}},
    {&driveMotor4, &steeringMotor4, &encoder4, {17.75, -25}}
  };

  // swerve drive object to control the 4-SwerveModule array using the motion controller object
  SwerveDrive swerve;
  SwervePoseTargeting swerveTargeting{&swerve};

  
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
