#pragma once
#include "translation2D/Pose.h"

// parameters for robot movement and autonomous
struct final
{
    double robotAccel = 0.04; // acceleration rate of the robot pose on the field
    double defaultAutoMaxDriveRate = 0.2;
    double defaultAutoMaxRotationRate = 0.2;

    // swerve presets
    Vector startingPosition = {0, 0};
    Angle startingAngle = 0;
    Pose startingPose = {startingPosition, startingAngle};
} parameters;