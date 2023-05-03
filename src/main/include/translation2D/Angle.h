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
     * return (left side).value + (right side).value
     * limited to -180 to 180 degrees
     * */
    Angle operator+(Angle const &obj)
    {
        Angle res;
        res.value = value + obj.value;
        res.value += res.value > 180 ? -360 : res.value < -180 ? 360
                                                               : 0;
        return res;
    }

    /**
     * (left side).value = (left side).value + (right side).value
     * then limit to -180 to 180 degrees
     * */
    void operator+=(Angle const &obj)
    {
        value += obj.value;
        value += value > 180 ? -360 : value < -180 ? 360
                                                   : 0;
    }

    /**
     * return (left side).value - (right side).value
     * limited to -180 to 180 degrees
     * */
    Angle operator-(Angle const &obj)
    {
        Angle res;
        res.value = value - obj.value;
        res.value += res.value > 180 ? -360 : res.value < -180 ? 360
                                                               : 0;
        return res;
    }

    /**
     * (left side).value = (left side).value - (right side).value
     * then limit to -180 to 180 degrees
     * */
    void operator-=(Angle const &obj)
    {
        value -= obj.value;
        value += value > 180 ? -360 : value < -180 ? 360
                                                   : 0;
    }

    // return (left side).value * (right side)
    Angle operator*(double const &k)
    {
        return Angle{value * k};
    }

    // (left side).value = (left side).value * (right side)
    void operator*=(double const &k)
    {
        value *= k;
    }

    // (left side).value = (left side).value / (right side)
    void operator/=(double const &k)
    {
        value /= k;
    }

    // move this Angle object toward another Angle object
    void moveToward(Angle target, double rate)
    {
        target -= *this;
        if (std::abs(target.value) > 2 * rate)
        {
            target /= std::abs(target.value);
            target *= rate;
            *this += target;
        }
        else
        {
            target /= 2;
            *this += target;
        }
    }
};

// returns the absolute value of an angle object's value
double abs(Angle obj)
{
    return std::abs(obj.value);
}