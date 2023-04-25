#pragma once
#include "AHRS.h"
#include "Parameters.h"
#include "SwerveModule.h"

/**
 * creates a swerve drive object that controls
 * an array of swerve drive modules
 * */
class SwerveDrive
{
private:
    Pose robotRate;
    Pose fieldRate;
    double moduleWheelSpeed;         // stores the velocity of each module in turn
    double fastestModule;            // fastest module velocity to be limited to 1
    Vector averagePositionChange;    // average module position change
    Vector fieldDisplacement{};      // field location in inches from the starting point
    AHRS navx{frc::SPI::Port::kMXP}; // NavX V2 object
    Angle navXAngle;
    Angle fieldAngle;

public:
    /**
     * runs the swerve modules using the values from the motion controller
     **/
    void Set(SwerveModule modules[4], Pose driveRate)
    {
        navXAngle = Angle{navx.GetYaw()};
        fieldAngle = navXAngle + params.startingAngle;

        robotRate = driveRate.getRotatedCCW(fieldAngle); // robot orient the drive rate

        fastestModule = 1;
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            moduleWheelSpeed = modules[i].getWheelSpeed(driveRate);
            if (moduleWheelSpeed > fastestModule)
            {
                fastestModule = moduleWheelSpeed;
            }
        }
        driveRate /= fastestModule;                         // limit the drive rate to keep all velocities below 1
        fieldRate.moveToward(driveRate, params.robotAccel); // accelerate toward the drive rate target
        robotRate = fieldRate.getRotatedCCW(fieldAngle);    // robot orient the drive rate

        averagePositionChange = Vector{}; // reset the average to zero before averaging again
        for (int i = 0; i < 4; i++)       // loop through the module indexes again
        {
            modules[i].Set(robotRate);                                                            // set each module using the accelerated robot rate
            averagePositionChange += modules[i].getwheelPositionChange().getRotatedCW(fieldAngle); // add the wheel velocity to the total sum
        }
        averagePositionChange /= 4; // find the average position change
        averagePositionChange *= 1 / 6.75 / 2048 * M_PI * 3.9;
        fieldDisplacement += averagePositionChange; // adds the distance traveled this cycle to the total distance to find the position
    }

    void zeroYaw()
    {
        navx.ZeroYaw();
    }

    Pose getFieldPose()
    {
        return Pose{fieldDisplacement, fieldAngle};
    }
};