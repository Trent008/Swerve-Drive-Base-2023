#pragma once
#include "math.h"
#include "cmath"

/**
 * stores 2D coordinates
 * can be added/subtacted with other Vector objects
 * or multiplied/divided by a double
*/
class Vector
{
public:
    double x, y; // coordinates

    /**
     * constructor for 2D vector
     * given x and y coordinates
     * */ 
    Vector(double x = 0, double y = 0)
    {
        this->x = x;
        this->y = y;
    }

    // add another vector to this vector and return the result
    Vector add(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
        return *this;
    }

    // subtract another vector from this vector and return the result
    Vector subtract(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
        return *this;
    }

    // return this vector after scaling by the given constant
    Vector scale(double const &k)
    {
        x *= k;
        y *= k;
        return *this;
    }

    // return this vector after dividing by the given constant
    Vector divide(double const &k)
    {
        x /= k;
        y /= k;
        return *this;
    }

    // return this vector's angle (-180 to 180 degrees)
    double getAngle()
    {
        return atan2(x, y) * 180 / M_PI;
    }

    // return this vector's magnitude as a double
    double getMagnitude()
    {
        return hypot(x, y);
    }

    // move this vector toward another vector's value by the specified increment
    void moveToward(Vector target, double increment)
    {
        target.subtract(*this);
        if (target.getMagnitude() > 2 * increment)
        {
            this->add(target.divide(target.getMagnitude()).scale(increment));
        }
        else
        {
            this->add(target.divide(2.0));
        }
    }

    // rotate this vector clockwise by the given angle
    Vector rotateCW(double angle)
    {
        angle *= M_PI / 180;
        *this = Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
        return *this;
    }

    // rotate this vector counter-clockwise by the given angle
    Vector rotateCCW(double angle)
    {
        angle *= M_PI / 180;
        *this = Vector{x * cos(angle) - y * sin(angle), y * cos(angle) + x * sin(angle)};
        return *this;
    }
};

// return the magnitude of the given vector as a double
double abs(Vector obj)
{
    return obj.getMagnitude();
}