#pragma once
#include "utilities.h"
#include "ctre/phoenix.h"
#include "ctre/phoenixpro/TalonFX.hpp"
#include "rev/CANSparkMax.h"

using namespace math;
using namespace ctre::phoenixpro;

class SwerveModule
{
private:
    Vector turnVector;         // vector corresponding to the way the rotation rate adds to the swerve module velocity
    Vector moduleVelocity;
    double wheelDirection; // direction of the wheel depending on whether the wheel is drivinging forward or backward
    double wheelSpeed;
    Angle error;
    double steeringMotorP; // proportional value determines how quickly the steering responds to angle setpoints
    double lastPosition = 0;
    double currentPosition;
    hardware::TalonFX *driveMotor;
    controls::PositionTorqueCurrentFOC *driveMotorOut;
    rev::CANSparkMax *steeringMotor;
    CANCoder *wheelEncoder;
    Vector wheelPositionChange;
    units::angle::turn_t rotations = 0_tr;

public:
    /**
     * parameters posX and posY set the position of
     * the module relative to the center of the robot
     */
    SwerveModule(hardware::TalonFX *driveMotor, controls::PositionTorqueCurrentFOC *driveMotorOut, rev::CANSparkMax *steeringMotor, CANCoder *wheelEncoder, Vector position = {})
    {
        steeringMotorP = 1;
        this->driveMotor = driveMotor;
        this->driveMotorOut = driveMotorOut;
        this->steeringMotor = steeringMotor;
        this->wheelEncoder = wheelEncoder;
        turnVector = position;
        turnVector /= abs(turnVector);
        turnVector.rotateCW(90);
    }

    double getWheelSpeed(Pose robotRate)
    {
        moduleVelocity = robotRate.getVector() + turnVector * robotRate.getAngle();
        return abs(moduleVelocity);
    }

    void Set(Pose robotRate)
    {
        moduleVelocity = robotRate.getVector() + turnVector * robotRate.getAngle();
        error = moduleVelocity.getAngle() - Angle{wheelEncoder->GetAbsolutePosition()};
        if (abs(error) < 90) {
            wheelDirection = 1;
        }
        else
        {
            error += 180;
            wheelDirection = -1;
        }
        wheelSpeed = abs(moduleVelocity) * wheelDirection;
        rotations += wheelSpeed * 2.3_tr;
        driveMotor->SetControl(driveMotorOut->WithPosition(rotations));
        steeringMotor->Set(double(error) * -steeringMotorP / 180);
        currentPosition = driveMotor->GetPosition().GetValue().value();
        wheelPositionChange = Vector{0, currentPosition - lastPosition};
        wheelPositionChange.rotateCW(wheelEncoder->GetAbsolutePosition());
        lastPosition = currentPosition;
    }

    Vector getwheelPositionChange()
    {
        return wheelPositionChange;
    }
};