#pragma once
#include "SwerveSetpoint.h"

// pose, armPose, useLimelight, driveRate, rotationRate
struct AutoSetpoint
{
    SwerveSetpoint swerveSetpoint;
    double driveRate = .21;
    double rotationRate = 0.21;
    bool useLimelight = false;
};