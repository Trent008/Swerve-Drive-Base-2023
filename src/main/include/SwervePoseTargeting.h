#pragma once
#include "SwerveDrive.h"
#include "SwerveSetpoint.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of current positions and angles
 **/
class SwervePoseTargeting
{
private:
    double positionProportional = 0.023; // rate at which to approach the current position
    double angleProportional = 0.007;    // rate at which to approach the current angle
    Pose poseError;              // how fast the robot needs to move to get to its next position setpoint
    Pose swerveRate;
    SwerveDrive *swerve;

public:
    SwervePoseTargeting(SwerveDrive *swerve)
    {   
        this->swerve = swerve;
    }

    void targetPose(SwerveModule moduleArray[4], SwerveSetpoint swerveSetpoint)
    {   
        poseError = swerveSetpoint.targetPose - swerve->getFieldPose();
        swerveRate = poseError;
        swerveRate *= Vector{positionProportional, angleProportional};
        swerveRate.limit(swerveSetpoint.driveRate, swerveSetpoint.rotationRate);
        swerve->Set(moduleArray, swerveRate);
    }

    bool poseReached(double positionTolerance, double angleTolerance)
    {
        return (abs(poseError.getVector()) < positionTolerance) && (abs(poseError.getAngle()) < angleTolerance);
    }
};