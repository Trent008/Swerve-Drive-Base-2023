#pragma once
#include "AHRS.h"
#include "SwerveModule.h"

class final {

private:
    // module array
    SwerveModule modules[4] =
    {
        SwerveModule{11, 31, 21, {-17.75, 25}},
        SwerveModule{12, 32, 22, {-17.75, -25}},
        SwerveModule{13, 33, 23, {17.75, 25}},
        SwerveModule{14, 34, 24, {17.75, -25}}
    };

    AHRS navx{frc::SPI::Port::kMXP}; // NavX V2 object

    Vector fieldDisplacement;
    Vector averageModuleChange;
    Vector currentFieldRate;
    float currentAngularRate = 0;
    float fieldAngle;
    float fastestModule;
    float moduleWheelSpeed;
    Vector positionalAccelOutput;
    float angularAccelOutput;


public:
    void initialize() {
        for (int i = 0; i < 4; i++) {
            modules[i].initialize();
        }
        navx.ZeroYaw();
    }

    void Set(Vector fieldRateTarget, float angularRateTarget, bool isAutonomous = false) {
        fieldAngle = angleSum(navx.GetYaw(), parameters.startingAngle);
        fieldRateTarget.rotateCW(-fieldAngle); // field orient the drive command
        normalizeSwerveRate(fieldRateTarget, angularRateTarget); // keeps the module speeds in range
        averageModuleChange = Vector{0, 0};
        currentFieldRate = Vector{0, 0};
        currentAngularRate = 0;
        for (int i = 0; i < 4; i++) // do some stuff for all four modules
        {
            averageModuleChange.add(modules[i].getPositionChangeVector()); // add the wheel velocity to the total sum
            currentFieldRate.add(modules[i].getCurrentVelocity()); // add the wheel velocity to the total sum
            currentAngularRate += modules[i].getRotationalProjection();
        }
        averageModuleChange.rotateCW(fieldAngle);
        averageModuleChange.scale(0.25);
        fieldDisplacement = fieldDisplacement.getAdded(averageModuleChange);
        currentFieldRate.rotateCW(fieldAngle);
        currentFieldRate.scale(0.25);
        currentAngularRate /= 4;

        // find the velocity error
        positionalAccelOutput = fieldRateTarget.getSubtracted(currentFieldRate).getDivided(2);
        if (t2D::abs(positionalAccelOutput) > parameters.maxPercentChangePerCycle) {
            positionalAccelOutput.scale(parameters.maxPercentChangePerCycle / t2D::abs(positionalAccelOutput));
        }
        angularAccelOutput = (angularRateTarget - currentAngularRate) / 2;
        if (std::abs(angularAccelOutput) > parameters.maxPercentChangePerCycle) {
            angularAccelOutput *= parameters.maxPercentChangePerCycle / std::abs(angularAccelOutput);
        }
        // increment from the current field rate
        currentFieldRate.add(positionalAccelOutput);
        currentAngularRate += angularAccelOutput;
        // get acceleration in amps
        positionalAccelOutput.scale(parameters.ampsForRobotAccel/parameters.maxPercentChangePerCycle);
        angularAccelOutput *= parameters.ampsForRobotAccel/parameters.maxPercentChangePerCycle;

        // drive the modules
        for (int i = 0; i < 4; i++) // do some stuff for all four modules
        {
            modules[i].Drive(currentFieldRate, currentAngularRate, positionalAccelOutput, angularAccelOutput);
        }
    }

    // limit the driving inputs to physically achievable values
    void normalizeSwerveRate(Vector &driveRate, float &angularRate) {
        fastestModule = 1;
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            moduleWheelSpeed = t2D::abs(modules[i].getModuleVector(driveRate, angularRate));
            if (moduleWheelSpeed > fastestModule)
            {
                fastestModule = moduleWheelSpeed;
            }
        }
        driveRate.divide(fastestModule);
        angularRate /= fastestModule;
    }
} swerve;