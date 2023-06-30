#pragma once
#include "Vector.h"
#include "ctre/phoenixpro/TalonFX.hpp"

// parameters for robot movement and autonomous
struct Parameters
{  
    float const ampsForRobotAccel = 50;
    float const wheelDiameter = 3.9;
    float const driveMotorInchesPerRotation = (M_PI * wheelDiameter / 6.75);
    float const falconMaxRotationsPerSecond = 101;
    float const maxPercentChangePerCycle = 0.035;
    float const defaultAutoMaxDriveRate = 0.2;
    float const defaultAutoMaxRotationRate = 0.2;

    // swerve presets
    Vector const startingPosition = {0, 0};
    float const startingAngle = 0;
} parameters;