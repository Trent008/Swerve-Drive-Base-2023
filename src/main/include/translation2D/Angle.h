#pragma once
#include "cmath"
#include "math.h"

/**
 * stores angular values from -180 to 180 degrees
 * can be added or subtracted with other angles without going out of bounds
 * can be multiplied or divided by a double
 * */
class Angle
{
public:
    double value; // size of the angle

    /**
     * angle object for storing angular values in degrees
     * 
     * angle is limited to -180 to 180 when added or subtracted
     * */
    Angle(double angle = 0)
    {
        value = angle;
    }

    /**
     * add another angle to this and return the result
     * limited to -180 to 180 degrees
     * */
    Angle add(Angle const &obj)
    {
        value += obj.value;
        value += value > 180 ? -360 : value < -180 ? 360
                                                   : 0;
        return *this;
    }

    /**
     * subtract another angle from this and return the result
     * limited to -180 to 180 degrees
     * */
    Angle subtract(Angle const &obj)
    {
        value -= obj.value;
        value += value > 180 ? -360 : value < -180 ? 360
                                                   : 0;
        return *this;
    }

    Angle getAdded(Angle const &obj)
    {
        Angle res = *this;
        res.value += obj.value;
        res.value += res.value > 180 ? -360 : res.value < -180 ? 360
                                                   : 0;
        return res;
    }

    Angle getSubtracted(Angle const &obj)
    {
        Angle res = *this;
        res.value -= obj.value;
        res.value += res.value > 180 ? -360 : res.value < -180 ? 360
                                                   : 0;
        return res;
    }


    Angle scale(double const &k)
    {
        value *= k;
        return *this;
    }

    Angle divide(double const &k)
    {
        value /= k;
        return *this;
    }

    // move this Angle object toward another Angle object
    void moveToward(Angle target, double rate)
    {
        target.subtract(*this);
        if (std::abs(target.value) > 2 * rate)
        {
            target.divide(std::abs(target.value));
            target.scale(rate);
            this->add(target);
        }
        else
        {
            target.divide(2);
            this->add(target);
        }
    }
};

// returns the absolute value of an angle object's value
double abs(Angle obj)
{
    return std::abs(obj.value);
}