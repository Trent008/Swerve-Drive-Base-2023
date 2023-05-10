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
    void add(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
    }

    Vector getAdded(Vector const &obj)
    {
        
        return Vector{x + obj.x, y + obj.y};
    }

    // subtract another vector from this vector
    void subtract(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
    }

    Vector getDifference(Vector const &obj)
    {
        
        return Vector{x - obj.x, y - obj.y};
    }

    // return this vector after scaling by the given constant
    void scale(double const &k)
    {
        x *= k;
        y *= k;
    }

    Vector getScaled(double const &k)
    {
        return Vector{x*k, y*k};
    }

    // return this vector after dividing by the given constant
    void divide(double const &k)
    {
        x /= k;
        y /= k;
    }

    Vector getDivided(double const &k)
    {
        return Vector{x/k, y/k};
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
            this->add(target.getDivided(target.getMagnitude()).getScaled(increment));
        }
        else// if (target.getMagnitude() > .005)
        {
            this->add(target.getDivided(2.0));
        }
    }

    // rotate this vector clockwise by the given angle
    Vector rotateCW(double angle)
    {
        angle *= M_PI / 180;
        *this = Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
        return *this;
    }

    Vector getRotatedCW(double angle)
    {
        angle *= M_PI / 180;
        return Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
    }
};

// return the magnitude of the given vector as a double
double abs(Vector obj)
{
    return obj.getMagnitude();
}