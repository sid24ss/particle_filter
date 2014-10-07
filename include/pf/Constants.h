#pragma once
#include <vector>
#include <numeric>

#include <pf/Utilities.h>

namespace pf {

    typedef std::vector<std::vector <double> > OccupancyGrid;

    class OccupancyState {
    public:
        enum {
            UNKNOWN = -1,
            OCCUPIED,   // 0
            FREE        // 1
        };
    };

    class RobotDOF {
    public:
        enum {
            X,
            Y,
            THETA
        };
    };
    
    struct MotionModelParams {
        static constexpr double alpha_1 = 1.0/30.0;
        // assumes distance in cm and angle in rad!
        static constexpr double alpha_2  = DEG2RAD(2)/(15);
        static constexpr double alpha_3 = 1.0/30.0;
        static constexpr double alpha_4 = 0.0;
    };

    namespace SensorModelParams {
        const double variance_scaling = 0.01;
        const double max_range = 1500.0;
        const double min_range = 0.0;
        inline std::vector<double> getBearings() {
            std::vector<double> bearings(180,0);
            std::iota(bearings.begin(), bearings.end(), -90);
            return bearings;
        }
        const double skip = 5;
    }

    namespace MapParams {
        // TODO: tune these parameters
        // Note : The Map is occupied if the prob_ value is 0
        // unoccupied if prob_ value is 1
        const double WALL_TOL = 0.1;
        const double FREE_TOL = 0.0;
    }
}
