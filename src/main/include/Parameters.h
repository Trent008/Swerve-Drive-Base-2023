#pragma once
#include "translations2D/Pose.h"
#include "ctre/phoenixpro/TalonFX.hpp"

// parameters for robot movement and autonomous
struct Parameters
{  
    double const robotAccelMetersPerSecondSquared = 10; // acceleration rate of the robot pose on the field
    double const ampsForRobotAccel = 50;
    double const wheelDiameter = 3.9;
    double const driveMotorInchesPerRotation = (M_PI * wheelDiameter / 6.75);
    double const falconMaxRotationsPerSecond = 101;
    double const swerveMaxRotationRate = falconMaxRotationsPerSecond * driveMotorInchesPerRotation / std::hypot(17.75, 25) * 180 / M_PI;
    double const maxRobotMetersPerSecond = falconMaxRotationsPerSecond * driveMotorInchesPerRotation * 0.0254;
    double const robotPercentChangePerCycle = (robotAccelMetersPerSecondSquared/maxRobotMetersPerSecond/50);
    double const defaultAutoMaxDriveRate = 0.2;
    double const defaultAutoMaxRotationRate = 0.2;

    // swerve presets
    Vector const startingPosition = {0, 0};
    Angle const startingAngle = 0;
    Pose const startingPose = {startingPosition, startingAngle};
} parameters;