#pragma once
#include "math.h"

class Vector {
public:
    float x, y; // coordinates

    /**
     * constructor for 2D vector
     * given x and y coordinates
     * */ 
    Vector(float x = 0, float y = 0)
    {
        this->x = x;
        this->y = y;
    }

    float radians(float const &degrees) {
        return degrees * M_PI / 180;
    }

    float degrees(float const &radians) {
        return radians * 180 / M_PI;
    }

    // return this vector's magnitude as a float
    float getMagnitude()
    {
        return hypot(x, y);
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

    Vector getSubtracted(Vector &obj)
    {
        
        return Vector{x - obj.x, y - obj.y};
    }

    // return this vector after scaling by the given constant
    void scale(float const &k)
    {
        x *= k;
        y *= k;
    }

    Vector getScaled(float const &k)
    {
        return Vector{x*k, y*k};
    }

    // return this vector after dividing by the given constant
    void divide(float const &k)
    {
        x /= k;
        y /= k;
    }

    Vector getDivided(float const &k)
    {
        return Vector{x/k, y/k};
    }

    // return this vector's angle (-180 to 180 degrees)
    float getAngle()
    {
        return degrees(atan2(x, y));
    }

    // move this vector toward another vector's value by the specified increment
    void moveToward(Vector target, float increment)
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
    void rotateCW(float angle)
    {
        angle = radians(angle);
        *this = Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
    }

    Vector getRotatedCW(float angle)
    {
        angle = radians(angle);
        return Vector{x * cos(angle) + y * sin(angle), y * cos(angle) - x * sin(angle)};
    }

    float getMagnitudeOfProjectionOnto(Vector &baseVector) {
        cos(radians(this->getAngle() - baseVector.getAngle())) * this->getMagnitude();
    }
};