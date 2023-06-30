#pragma once
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include "Falcon500.h"
#include "translations2D.h"

class SwerveModule {
private:
    Vector turnVector;
    Falcon500 *driveMotor;
    rev::CANSparkMax *turningMotor;
    CANCoder *wheelAngleEncoder;
    float lastPosition = 0;
    Vector positionChangeVector;
    Vector currentVelocity;
    float currentWheelAngle;
    Vector moduleTargetVelocity;
    float error;
    float driveMotorAccel;
    float currentPosition;

public:
    SwerveModule(int driveMotorCANID, int turningMotorCANID, int canCoderID, Vector position) {
        driveMotor = new Falcon500{driveMotorCANID};
        turningMotor = new rev::CANSparkMax{turningMotorCANID, rev::CANSparkMax::MotorType::kBrushless};
        wheelAngleEncoder = new CANCoder{canCoderID};
        turnVector = position;
        turnVector.rotateCW(90);
        turnVector.divide(t2D::abs(turnVector));
    }

    void initialize() {
        driveMotor->initialize();
        turningMotor->SetInverted(true);
        turningMotor->BurnFlash();
    }

    Vector getModuleVector(Vector driveRate, float angularRate)
    {
        return driveRate.getAdded(turnVector.getScaled(angularRate));
    }

    void Drive(Vector driveRate, float angularRate, Vector positionalAcceleration, float angularAcceleration) {
        currentWheelAngle = wheelAngleEncoder->GetAbsolutePosition();
        moduleTargetVelocity = getModuleVector(driveRate, angularRate);
        error = angleDifference(moduleTargetVelocity.getAngle(), currentWheelAngle);
        driveMotorAccel = getModuleVector(positionalAcceleration, angularAcceleration).getMagnitudeOfProjectionOnto(moduleTargetVelocity);
        if (std::abs(error) > 90)
        {
            driveMotorAccel = -driveMotorAccel;
            error = angleSum(error, 180);
        }
        driveMotor->SetAcceleration(driveMotorAccel);
        turningMotor->Set(error / 180);

        // find the delta position change since last Set() call
        currentPosition = driveMotor->getPosition();
        positionChangeVector = Vector{0, (currentPosition - lastPosition) * parameters.driveMotorInchesPerRotation}.getRotatedCW(currentWheelAngle);
        lastPosition = currentPosition;
        currentVelocity = Vector{0, driveMotor->getPercentOfMaxVelocity()}.getRotatedCW(currentWheelAngle);
    }

    Vector getPositionChangeVector() {
        return positionChangeVector;
    }

    Vector getCurrentVelocity() {
        return currentVelocity;
    }

    float getRotationalProjection() {
        return getCurrentVelocity().getMagnitudeOfProjectionOnto(turnVector);
    }
};