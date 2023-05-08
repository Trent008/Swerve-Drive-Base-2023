#pragma once
#include "translations2D/Pose.h"
#include "ctre/phoenix.h"
#include "ctre/phoenixpro/TalonFX.hpp"
#include "rev/CANSparkMax.h"
#include "Parameters.h"

using namespace ctre::phoenixpro;

class SwerveModule
{
private:
    Vector turnVector;         // vector corresponding to the way the rotation rate adds to the swerve module velocity
    Vector moduleVelocity;     // stores this modules velocity vector
    Vector driveRate;
    Angle steeringRate;
    Angle error;
    double lastPosition = 0;
    double currentPosition;
    hardware::TalonFX *driveMotor;
    controls::VelocityTorqueCurrentFOC driveMotorCTRL{0_tr, 0_A, 0, false};
    rev::CANSparkMax *steeringMotor;
    CANCoder *wheelEncoder;
    Vector wheelPositionChange;
    // units::angle::turn_t driveRotations = 0_tr;

public:
    /**
     * parameters posX and posY set the position of
     * the module relative to the center of the robot
     */
    SwerveModule(int driveMotorID, int steeringMotorID, int wheelEncoderID, Vector position = {})
    {
        driveMotor = new hardware::TalonFX(driveMotorID, "rio");

        steeringMotor = new rev::CANSparkMax(steeringMotorID, rev::CANSparkMax::MotorType::kBrushless);
        
        wheelEncoder = new CANCoder(wheelEncoderID);

        turnVector = position;
        turnVector.divide(abs(turnVector));
        turnVector.rotateCW(90);
    }

    void initialize()
    {   
        configs::TalonFXConfiguration configs{};
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
        configs.TorqueCurrent.PeakForwardTorqueCurrent = parameters.ampsForRobotAccel;  // Peak output of 40 amps
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -parameters.ampsForRobotAccel; // Peak output of 40 amps
        driveMotor->GetConfigurator().Apply(configs);
        driveMotor->SetRotorPosition(0_tr);

        steeringMotor->RestoreFactoryDefaults();
        steeringMotor->SetInverted(true);
        //steeringPID->SetP(0.1);
        //steeringPID->SetI(1e-4);
        //steeringPID->SetD(1);
        //steeringPID->SetOutputRange(-1, 1);
        //steeringPID->SetPositionPIDWrappingEnabled(true);
        //steeringPID->SetPositionPIDWrappingMinInput(-180);
        //steeringPID->SetPositionPIDWrappingMaxInput(180);
        //steeringEncoder->SetPositionConversionFactor(360/12.8);
        //steeringEncoder->SetPosition(wheelEncoder->GetAbsolutePosition());
    }

    Vector getModuleVector(Pose robotRate)
    {
        driveRate = robotRate.vector;
        steeringRate = robotRate.angle;
        return driveRate.add(turnVector.getScaled(steeringRate.value));
    }

    void Set(Pose robotRate)
    {
        moduleVelocity = getModuleVector(robotRate);
        error = Angle{moduleVelocity.getAngle()}.getSubtracted(wheelEncoder->GetAbsolutePosition());
        auto frictionTorque = 1_A;
        auto driveRate = abs(moduleVelocity) * parameters.falconMaxRotationsPerSecond;
        if (abs(error) > 90)
        {
            frictionTorque = -frictionTorque;
            driveRate = -driveRate;
        }
        driveMotor->SetControl(driveMotorCTRL.WithVelocity(driveRate).WithFeedForward(frictionTorque));
        
        steeringMotor->Set(error.value / 180);
        // steeringPID->SetReference(targetAngle.value, rev::CANSparkMax::ControlType::kPosition);

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