#pragma once
#include "translations2D/Pose.h"
#include "ctre/phoenixpro/TalonFX.hpp"

// parameters for robot movement and autonomous
struct Parameters
{  
    double static const robotAccelMetersPerSecondSquared = 8; // acceleration rate of the robot pose on the field
    double static const ampsForRobotAccel = 40;
    double static const wheelDiameter = 3.9;
    double static const driveMotorInchesPerRotation = (M_PI * wheelDiameter / 6.75);
    double static const falconMaxRotationsPerSecond = 101;
    double const swerveMaxRotationRate = falconMaxRotationsPerSecond * driveMotorInchesPerRotation / std::hypot(17.75, 25) * 180 / M_PI;
    double static const maxRobotMetersPerSecond = falconMaxRotationsPerSecond * driveMotorInchesPerRotation * 0.0254;
    double static const robotPercentChangePerCycle = (robotAccelMetersPerSecondSquared/maxRobotMetersPerSecond/50);
    double const defaultAutoMaxDriveRate = 0.2;
    double const defaultAutoMaxRotationRate = 0.2;

    // swerve presets
    Vector const startingPosition = {0, 0};
    Angle const startingAngle = 0;
    Pose const startingPose = {startingPosition, startingAngle};
} parameters;