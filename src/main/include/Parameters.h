#pragma once
#include "AutoSetpoint.h"

// scroll to bottom for autonomous setpoints

// parameters for robot movement and autonomous
struct Parameters
{
    // set ramp
    double robotAccel = 0.04; // acceleration rate of the robot pose on the field

    // swerve presets
    Vector startingPosition = {0, 0};
    Angle startingAngle = 0;
    Pose startingPose = {startingPosition, startingAngle};

    /*
     * the pose, armPosition, wrist, suction, useLimelight, driveRate, rotationRate
     * for the autonomous routine
     */
    AutoSetpoint setpoints[15] =
        {
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}},
            {{startingPose}}};
} params;