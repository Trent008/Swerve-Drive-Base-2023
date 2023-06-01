#pragma once
#include "AHRS.h"
#include "Parameters.h"
#include "SwerveModule.h"

/**
 * creates a swerve drive object that controls
 * an array of swerve drive modules
 * */
class final
{
private:
    // module array
    SwerveModule modules[4] =
    {
        SwerveModule{11, 31, 21, {-17.75, 25}},
        SwerveModule{12, 32, 22, {-17.75, -25}},
        SwerveModule{13, 33, 23, {17.75, 25}},
        SwerveModule{14, 34, 24, {17.75, -25}}
    };

    Pose robotRate; // stores the target robot-centric drive rate then the actual robot-cnetric drive rate
    Pose fieldRate; // field-centric rate after acceleration
    Pose fieldAccelCommand;
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
    void Set(Pose driveRate, bool isAutonomous = false, bool isRobotOriented = false)
    {
        navXAngle = Angle{navx.GetYaw()};
        fieldAngle = navXAngle.getAdded(parameters.startingAngle);
        if (isRobotOriented) {
            driveRate = driveRate.getRotatedCW(fieldAngle.value);
        }

        robotRate = driveRate.getRotatedCW(-fieldAngle.value); // robot orient the drive rate

        fastestModule = 1;
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            moduleWheelSpeed = abs(modules[i].getModuleVector(driveRate));
            if (moduleWheelSpeed > fastestModule)
            {
                fastestModule = moduleWheelSpeed;
            }
        }
        driveRate.divide(fastestModule);                         // limit the drive rate to keep all velocities below 1
        if (!isAutonomous)
        {
            fieldRate.moveToward(driveRate, parameters.robotPercentChangePerCycle); // accelerate toward the drive rate target
        }
        else
        {
            fieldRate = driveRate;
        }
        robotRate = fieldRate.getRotatedCW(-fieldAngle.value);    // robot orient the drive rate

        averagePositionChange = Vector{}; // reset the average to zero before averaging again
        for (int i = 0; i < 4; i++)       // loop through the module indexes again
        {
            modules[i].Set(robotRate);                                                            // set each module using the accelerated robot rate
            averagePositionChange.add(modules[i].getwheelPositionChange().rotateCW(fieldAngle.value)); // add the wheel velocity to the total sum
        }
        averagePositionChange.divide(4); // find the average position change
        averagePositionChange.scale(parameters.driveMotorInchesPerRotation); // find the average and convert to inches
        fieldDisplacement.add(averagePositionChange); // adds the distance traveled this cycle to the total distance to find the position
    }

    void initialize()
    {
        for (int i = 0; i<4; i++)
        {
            modules[i].initialize();
        }
        zeroYaw();
    }

    void zeroYaw()
    {
        navx.ZeroYaw();
    }

    Pose getFieldPose()
    {
        return Pose{fieldDisplacement, fieldAngle};
    }
} swerve;