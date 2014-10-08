#pragma once
#include <cmath>
#include <algorithm>
#include <cassert>

#define RAD2DEG(th) th*180/M_PI
#define DEG2RAD(th) th*M_PI/180

namespace pf {

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

inline void normalizeData(std::vector<double>& data)
{
    double sum_of_weights = 0.0;
    std::for_each(data.begin(), data.end(),
        [&sum_of_weights](double val){sum_of_weights += val; }
    );
    assert(sum_of_weights != 0.0);
    std::for_each(data.begin(), data.end(),
        [sum_of_weights](double& val){ val /= sum_of_weights; }
    );
}

} // pf
