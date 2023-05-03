#pragma once
#include "Swerve/SwervePreset.h"

// chassis target pose, max drive rate, and max rotation rate
struct AutonomousSetpoint
{
    SwervePreset swerveSetpoint = SwervePreset{}; // stores variables required to drive the robot to a new pose
    
    // include any other variables that need to be changed during the autonomous routine
};