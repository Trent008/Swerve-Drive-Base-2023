#pragma once
#include "translations2D/Pose.h"
#include "Parameters.h"

// stores variables needed to drive to a pose on the field
struct SwervePreset
{
    Pose targetPose = parameters.startingPose;                      // target position and angle for the SwervePoseController
    double maxDriveRate = parameters.defaultAutoMaxDriveRate;       // max drive rate for SwervePoseController
    double maxRotationRate = parameters.defaultAutoMaxRotationRate; // max rotation rate for SwervePoseController
};