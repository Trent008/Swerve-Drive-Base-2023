#pragma once
#include "cmath"
#include "math.h"

namespace math
{
    class Angle
    {
    private:
        double a;

    public:
        Angle(double angle = 0)
        {
            a = angle;
        }

        operator double() const
        {
            return a;
        }

        Angle operator+(Angle const &obj)
        {
            Angle res;
            res.a = a + obj.a;
            res.a += res.a > 180 ? -360 : res.a < -180 ? 360
                                        : 0;
            return res;
        }

        void operator+=(Angle const &obj)
        {
            a += obj.a;
            a += a > 180 ? -360 : a < -180 ? 360
                                        : 0;
        }

        Angle operator-(Angle const &obj)
        {
            Angle res;
            res.a = a - obj.a;
            res.a += res.a > 180 ? -360 : res.a < -180 ? 360
                                        : 0;
            return res;
        }

        void operator-=(Angle const &obj)
        {
            a -= obj.a;
            a += a > 180 ? -360 : a < -180 ? 360
                                        : 0;
        }

        Angle operator*(Angle const &obj)
        {
            return Angle{a * obj.a};
        }

        void operator*=(double const &value)
        {
            a *= value;
        }

        void operator/=(double const &value)
        {
            a /= value;
        }
        
        void moveToward(Angle target, double rate)
        {
            target -= *this;
            if (std::abs(double(target)) > 2 * rate)
            {
                target /= std::abs(double(target));
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

    double abs(Angle obj)
    {
        return std::abs(double(obj));
    }

    class Vector
    {
    private:
        double x, y;

    public:

    
        Vector(double x = 0, double y = 0)
        {
            this->x = x;
            this->y = y;
        }

        operator double() const
        {
            return std::hypot(x, y);
        }

        Vector operator+(Vector const &obj)
        {
            return Vector{x + obj.x, y + obj.y};
        }

        void operator+=(Vector const &obj)
        {
            x += obj.x;
            y += obj.y;
        }

        Vector operator-(Vector const &obj)
        {
            return Vector{x - obj.x, y - obj.y};
        }

        void operator-=(Vector const &obj)
        {
            x -= obj.x;
            y -= obj.y;
        }

        Vector operator*(double const &value)
        {
            return Vector{x*value, y*value};
        }

        void operator*=(double const &value)
        {
            x *= value;
            y *= value;
        }

        Vector operator/(double const &value)
        {
            return Vector{x/value, y/value};
        }

        void operator/=(double const &value)
        {
            x /= value;
            y /= value;
        }

        double getX() {
            return x;
        }

        double getY() {
            return y;
        }

        void setX(double const &x)
        {
            this->x = x;
        }

        void setY(double const &y)
        {
            this->y = y;
        }
        
        void moveToward(Vector target, double rate)
        {
            target -= *this;
            if (double(target) > 2 * rate)
            {
                *this += target / double(target) * rate;
            }
            else
            {
                *this += target / 2.0;
            }
        }

        Angle getAngle()
        {
            return Angle{atan2(x, y) * 180 / M_PI};
        }

        void rotateCW(Angle const &angle)
        {
            *this = this->getRotatedCW(angle);
        }

        Vector getRotatedCW(Angle obj)
        {
            obj *= M_PI / 180;
            return Vector{x*cos(double(obj)) + y*sin(double(obj)), y*cos(double(obj)) - x*sin(double(obj))};
        }

        void rotateCCW(Angle const &angle)
        {
            *this = this->getRotatedCCW(angle);
        }

        Vector getRotatedCCW(Angle obj)
        {
            obj *= M_PI / 180;
            return Vector{x*cos(double(obj)) - y*sin(double(obj)), y*cos(double(obj)) + x*sin(double(obj))};
        }
    };

    double abs(Vector obj)
    {
        return std::hypot(obj.getX(), obj.getY());
    }

    class Pose
    {
    private:
        Vector vector;
        Angle angle;
    public:
        Pose(Vector vector = Vector{}, Angle angle = Angle{})
        {
            this->vector = vector;
            this->angle = angle;
        }

        void operator+=(Pose obj)
        {
            vector += obj.vector;
            angle += obj.angle;
        }

        Pose operator+(Pose obj)
        {
            Pose res;
            res += obj;
            return res;
        }

        void operator-=(Pose pose)
        {
            vector -= pose.vector;
            angle -= pose.angle;
        }

        Pose operator-(Pose obj)
        {
            Pose res;
            res -= obj;
            return res;
        }

        void operator*=(Vector obj)
        {
            this->vector *= vector.getX();
            this->angle *= vector.getY();
        }

        void operator/=(double divisor)
        {
            vector /= divisor;
            angle /= divisor;
        }

        void moveToward(Pose pose, double rate) {
            vector.moveToward(pose.vector, rate);
            angle.moveToward(pose.angle, rate);
        }

        double magnitude()
        {
            return abs(vector) + abs(angle);
        }

        Vector getVector()
        {
            return vector;
        }

        Angle getAngle()
        {
            return angle;
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

        Pose getRotatedCW(Angle angle)
        {
            return Pose{vector.getRotatedCW(angle), this->angle};
        }

        Pose getRotatedCCW(Angle angle)
        {
            return Pose{vector.getRotatedCCW(angle), this->angle};
        }
    };
}