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

    Pose getAdded(Pose const &obj)
    {
        Pose res = *this;
        res.vector.add(obj.vector);
        res.angle.add(obj.angle);
        return res;
    }

    Pose getDifference(Pose const &obj)
    {
        Pose res = *this;
        res.vector.subtract(obj.vector);
        res.angle.subtract(obj.angle);
        return res;
    }

    void scale(double kPositional, double kAngular)
    {
        vector.scale(kPositional);
        angle.scale(kAngular);
    }

    void divide(double k)
    {
        vector.divide(k);
        angle.divide(k);
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
            vector.scale(vectorLimit / abs(vector));
        }
        if (abs(angle) > angleLimit)
        {
            angle.scale(angleLimit / abs(angle));
        }
    }

    Pose getRotatedCW(double angle)
    {
        return Pose{vector.rotateCW(angle), this->angle};
    }

    Pose getRotatedCCW(double angle)
    {
        return Pose{vector.rotateCCW(angle), this->angle};
    }
};