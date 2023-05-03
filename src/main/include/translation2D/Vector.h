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

    // return Vector{(left side).x + (right side).x, (left side).y + (right side).y}
    Vector operator+(Vector const &obj)
    {
        return Vector{x + obj.x, y + obj.y};
    }

    // return Vector{(left side).x - (right side).x, (left side).y - (right side).y}
    Vector operator-(Vector const &obj)
    {
        return Vector{x - obj.x, y - obj.y};
    }

    /**
     * (left side).x += (right side).x
     * (left side).y += (right side).y
    */
    void operator+=(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
    }

    /**
     * (left side).x -= (right side).x
     * (left side).y -= (right side).y
    */
    void operator-=(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
    }

    // return Vector (left side) * double (right side)
    Vector operator*(double const &k)
    {
        return Vector{x * k, y * k};
    }

    // return Vector (left side) / double (right side)
    Vector operator/(double const &k)
    {
        return Vector{x / k, y / k};
    }

    // Vector (left side) *= double (right side)
    void operator*=(double const &k)
    {
        x *= k;
        y *= k;
    }

    // Vector (left side) /= double (right side)
    void operator/=(double const &k)
    {
        x /= k;
        y /= k;
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
        target -= *this;
        if (target.getMagnitude() > 2 * increment)
        {
            *this += target / target.getMagnitude() * increment;
        }
        else
        {
            *this += target / 2.0;
        }
    }

    // rotate this vector clockwise by the given angle
    void rotateCW(double angle)
    {
        *this = this->getRotatedCW(angle);
    }

    // rotate this vector counter-clockwise by the given angle
    void rotateCCW(double angle)
    {
        *this = this->getRotatedCCW(angle);
    }

    // return this vector rotated clockwise by the given angle
    Vector getRotatedCW(double angle)
    {
        angle *= M_PI / 180;
        return Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
    }

    // return this vector rotated counter-clockwise by the given angle
    Vector getRotatedCCW(double angle)
    {
        angle *= M_PI / 180;
        return Vector{x * cos(angle) - y * sin(angle), y * cos(angle) + x * sin(angle)};
    }
};

// return the magnitude of the given vector as a double
double abs(Vector obj)
{
    return obj.getMagnitude();
}