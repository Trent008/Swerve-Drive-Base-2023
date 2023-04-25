#pragma once
#include "frc/Joystick.h"
#include "utilities.h"
using namespace math;

class XBOXController
{
private:
    frc::Joystick *joy;
    double nominalSpeed = 0.15;
    double nominalRotationRate = 0.1;
    double speed = 0.8;
    double rotationSpeed = 0.4;
    double deadband = 0.03;
    double a;

public:
    XBOXController(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    double getSpeed()
    {
        return nominalSpeed + joy->GetRawAxis(3) * (speed - nominalSpeed);
    }

    double getRotationSpeed()
    {
        return nominalRotationRate + joy->GetRawAxis(3) * (rotationSpeed - nominalRotationRate);
    }

    double getLX()
    {
        a = getSpeed() * joy->GetRawAxis(0);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);

        return a;
    }

    double getLY()
    {
        a = getSpeed() * -joy->GetRawAxis(1);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);
        return a;
    }

    // get Z Rotate axis
    double getRX()
    {
        a = getRotationSpeed() * joy->GetRawAxis(4);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);
        return a;
    }

    double getRY()
    {
        a = getSpeed() * -joy->GetRawAxis(5);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);
        return a;
    }

    Pose getFieldVelocity()
    {
        return Pose{Vector{getLX(), getLY()}, getRX()};
    }

    bool zero()
    {
        return joy->GetRawButton(1) && joy->GetRawButton(2) && joy->GetRawButton(3) && joy->GetRawButton(4);
    }
};