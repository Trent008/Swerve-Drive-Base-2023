#pragma once
#include "frc/Joystick.h"
#include "math.h"
#include "cmath"

class SpaceMouseEnt
{
private:
    frc::Joystick *joy;
    double a;

public:
    SpaceMouseEnt(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    double getX()
    {
        a = joy->GetRawAxis(0);
        return .5 * (std::abs(a) > 0.03 ? a : 0);
    }

    double getY()
    {
        a = joy->GetRawAxis(1);
        return 0.5 * (std::abs(a) > 0.03 ? a : 0);
    }

    double getZ()
    {
        a = joy->GetRawAxis(2);
        return -0.5 * (std::abs(a) > 0.03 ? a : 0);
    }

    double getXR()
    {
        a = -joy->GetRawAxis(3);
        return 2 * (std::abs(a) > 0.03 ? a : 0);
    }

    double getYR()
    {
        a = joy->GetRawAxis(4);
        return 1.5 * (std::abs(a) > 0.5 ? a : 0);
    }

    double getZR()
    {
        a = joy->GetRawAxis(5);
        return 2 * (std::abs(a) > 0.8 ? a : 0);
    }
};