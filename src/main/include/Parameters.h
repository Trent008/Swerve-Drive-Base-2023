#pragma once
#include "translations2D/Pose.h"
#include "ctre/phoenixpro/TalonFX.hpp"

// parameters for robot movement and autonomous
struct Parameters
{  
    double const robotAccelMetersPerSecondSquared = 8; // acceleration rate of the robot pose on the field
    double const ampsForRobotAccel = 40;
    double const wheelDiameter = 3.9;
    double const driveMotorRotationsToInches = (M_PI * wheelDiameter / 6.75);
    units::angular_velocity::turns_per_second_t const falconMaxRotationsPerSecond = 101_tps;
    double const maxRobotMetersPerSecond = falconMaxRotationsPerSecond.value() * driveMotorRotationsToInches * 0.0254;
    double const robotPercentChangePerCycle = robotAccelMetersPerSecondSquared/maxRobotMetersPerSecond/50;
    double const defaultAutoMaxDriveRate = 0.2;
    double const defaultAutoMaxRotationRate = 0.2;

    // swerve presets
    Vector const startingPosition = {0, 0};
    Angle const startingAngle = 0;
    Pose const startingPose = {startingPosition, startingAngle};
} parameters;