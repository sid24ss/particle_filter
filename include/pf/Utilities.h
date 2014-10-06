#pragma once
#include <cmath>
#define RAD2DEG(th) th*180/M_PI
#define DEG2RAD(th) th*M_PI/180

static inline double normalize_angle_positive(double angle)
{
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}

static inline double normalize_angle(double angle)
{
    double a = normalize_angle_positive(angle);
    if (a > M_PI)
        a -= 2.0 *M_PI;
    return a;
}

static inline double shortest_angular_distance(double from, double to)
{
    double result = normalize_angle_positive(normalize_angle_positive(to) - normalize_angle_positive(from));
    if (result > M_PI)
        // If the result > 180,
        // It's shorter the other way.
        result = -(2.0*M_PI - result);
    return normalize_angle(result);
}

