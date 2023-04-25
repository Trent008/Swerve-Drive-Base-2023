#pragma once
#include "utilities.h"
using namespace math;

struct SwerveSetpoint
{
    Pose targetPose = {};
    double driveRate = .21;
    double rotationRate = 0.21;
    bool useLimelight = false;
};
