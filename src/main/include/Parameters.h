#pragma once
#include "translations2D/Pose.h"

// parameters for robot movement and autonomous
struct final
{
    double const robotAccel = 0.04; // acceleration rate of the robot pose on the field
    double const defaultAutoMaxDriveRate = 0.2;
    double const defaultAutoMaxRotationRate = 0.2;
    double const driveMotorRotationsToInches = M_PI * 3.9 / 6.75;

    // swerve presets
    Vector const startingPosition = {0, 0};
    Angle const startingAngle = 0;
    Pose const startingPose = {startingPosition, startingAngle};
} parameters;