#pragma once
#include "SwerveDrive.h"
#include "SwervePreset.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of current positions and angles
 **/
class SwervePoseController
{
private:
    double positionProportional = 0.023; // rate at which to approach the current position
    double angleProportional = 0.007;    // rate at which to approach the current angle
    Pose poseError;              // how fast the robot needs to move to get to its next position setpoint
    Pose swerveRate;
    SwerveDrive *swerve;

public:
    SwervePoseController(SwerveDrive *swerve)
    {
        this->swerve = swerve;
    }

    void setReferencePose(SwervePreset setpoint)
    {   
        poseError = setpoint.targetPose - swerve->getFieldPose();
        swerveRate = poseError;
        swerveRate *= Vector{positionProportional, angleProportional};
        swerveRate.limit(setpoint.maxDriveRate, setpoint.maxRotationRate);
        swerve->Set(swerveRate);
    }

    bool poseReached(double positionTolerance, double angleTolerance)
    {
        return (abs(poseError.vector) < positionTolerance) && (abs(poseError.angle) < angleTolerance);
    }
};