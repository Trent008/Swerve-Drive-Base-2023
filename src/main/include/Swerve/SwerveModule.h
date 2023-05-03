#pragma once
#include "translations2D/Pose.h"
#include "ctre/phoenix.h"
#include "ctre/phoenixpro/TalonFX.hpp"
#include "rev/CANSparkMax.h"

using namespace ctre::phoenixpro;

class SwerveModule
{
private:
    Vector turnVector;         // vector corresponding to the way the rotation rate adds to the swerve module velocity
    Vector moduleVelocity;     // stores this modules velocity vector
    Angle targetAngle;
    double lastPosition = 0;
    double currentPosition;
    hardware::TalonFX *driveMotor;
    controls::PositionTorqueCurrentFOC driveMotorCTRL{0_tr, 0_A, 0, false};
    rev::CANSparkMax *steeringMotor;
    rev::SparkMaxPIDController *steeringPID;
    rev::SparkMaxRelativeEncoder *steeringEncoder;

    CANCoder *wheelEncoder;
    Vector wheelPositionChange;
    units::angle::turn_t driveRotations = 0_tr;
    double steeringRotations = 0;

public:
    /**
     * parameters posX and posY set the position of
     * the module relative to the center of the robot
     */
    SwerveModule(int driveMotorID, int steeringMotorID, int wheelEncoderID, Vector position = {})
    {
        driveMotor = new hardware::TalonFX(driveMotorID, "rio");

        steeringMotor = new rev::CANSparkMax(steeringMotorID, rev::CANSparkMax::MotorType::kBrushless);
        *steeringPID = steeringMotor->GetPIDController();
        *steeringEncoder = steeringMotor->GetEncoder();
        
        wheelEncoder = new CANCoder(wheelEncoderID);

        turnVector = position;
        turnVector.divide(abs(turnVector)).rotateCW(90);
    }

    void initialize()
    {   
        configs::TalonFXConfiguration configs{};
        configs.Slot0.kP = 40; // An error of 1 rotations results in 40 amps output
        configs.Slot0.kD = 2; // A change of 1 rotation per second results in 2 amps output
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;  // Peak output of 130 amps
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -130; // Peak output of 130 amps
        driveMotor->GetConfigurator().Apply(configs);
        driveMotor->SetRotorPosition(0_tr);

        steeringMotor->RestoreFactoryDefaults();
        steeringMotor->SetInverted(true);
        steeringPID->SetP(0.1);
        steeringPID->SetI(1e-4);
        steeringPID->SetD(1);
        steeringPID->SetOutputRange(-1, 1);
        steeringPID->SetPositionPIDWrappingEnabled(true);
        steeringPID->SetPositionPIDWrappingMinInput(-180);
        steeringPID->SetPositionPIDWrappingMaxInput(180);
        steeringEncoder->SetPositionConversionFactor(360/12.8);
        steeringEncoder->SetPosition(wheelEncoder->GetAbsolutePosition());
    }

    Vector getModuleVector(Pose robotRate)
    {
        return robotRate.vector.add(turnVector.scale(robotRate.angle.value));
    }

    void Set(Pose robotRate)
    {
        moduleVelocity = getModuleVector(robotRate);
        targetAngle = moduleVelocity.getAngle();
        if (abs(targetAngle.getSubtracted(wheelEncoder->GetAbsolutePosition())) < 90) {
            driveRotations += abs(moduleVelocity) * 2.3_tr; // scale position change rate so velociy hits max
        }
        else
        {   
            driveRotations -= abs(moduleVelocity) * 2.3_tr; // scale position change rate so velociy hits max
            targetAngle.add(180);
        }
        driveMotor->SetControl(driveMotorCTRL.WithPosition(driveRotations));
        steeringPID->SetReference(targetAngle.value, rev::CANSparkMax::ControlType::kPosition);

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