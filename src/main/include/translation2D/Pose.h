#pragma once
#include "Vector.h"
#include "Angle.h"


/**
 * stores a Vector object for a positional value
 * and an Angle object for an angular value
*/
class Pose
{
public:
    Vector vector; // stores the positional value of the pose
    Angle angle;   // stores the angular value of the pose
    
    Pose(Vector vector = Vector{}, Angle angle = Angle{})
    {
        this->vector = vector;
        this->angle = angle;
    }

    Pose operator+(Pose obj)
    {
        return Pose{vector + obj.vector, angle + obj.angle};
    }

    void operator+=(Pose obj)
    {
        vector += obj.vector;
        angle += obj.angle;
    }

    Pose operator-(Pose obj)
    {
        return Pose{vector - obj.vector, angle - obj.angle};
    }

    void operator-=(Pose pose)
    {
        vector -= pose.vector;
        angle -= pose.angle;
    }

    void operator*=(Vector obj)
    {
        vector *= obj.x;
        angle *= obj.y;
    }

    void operator/=(double k)
    {
        vector /= k;
        angle /= k;
    }

    void moveToward(Pose pose, double rate)
    {
        vector.moveToward(pose.vector, rate);
        angle.moveToward(pose.angle, rate);
    }

    void limit(double vectorLimit, double angleLimit)
    {
        if (abs(vector) > vectorLimit)
        {
            vector *= vectorLimit / abs(vector);
        }
        if (abs(angle) > angleLimit)
        {
            angle *= angleLimit / abs(angle);
        }
    }

    Pose getRotatedCW(double angle)
    {
        return Pose{vector.getRotatedCW(angle), this->angle};
    }

    Pose getRotatedCCW(double angle)
    {
        return Pose{vector.getRotatedCCW(angle), this->angle};
    }
};