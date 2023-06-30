#pragma once
#include "Vector.h"
#include "math.h"

namespace t2D {
    float abs(Vector const &obj) {
        return std::hypot(obj.x, obj.y);
    }
}

float getAngle(Vector const &obj) {
    return std::atan2(obj.x, obj.y) * 180 / M_PI;
}

float angleSum(float const &angle1, float const &angle2) {
    float res = angle1 + angle2;
    res += (res<-180) ? 360 : (res>180) ? -360 : 0;
    return res;
}

float angleDifference(float const &angle1, float const &angle2) {
    float res = angle1 - angle2;
    res += (res<-180) ? 360 : (res>180) ? -360 : 0;
    return res;
}

void incrementFloatTowardFloat(float &current, float const &target, float increment) {
    float error = target - current;
    if (std::abs(error) > 2 * increment)
    {
        current = current + error/std::abs(error);
    }
    else// if (target.getMagnitude() > .005)
    {
        current = current + error / 2;
    }
}

void incrementAngleTowardAngle(float &current, float const &target, float increment) {
    float error = angleDifference(target, current);
    if (std::abs(error) > 2 * increment)
    {
        current = angleSum(current, error/std::abs(error));
    }
    else// if (target.getMagnitude() > .005)
    {
        current = angleSum(current, error / 2);
    }
}