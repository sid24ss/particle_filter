#pragma once
#include <vector>

#include <pf/Utilities.h>

namespace pf {

    typedef std::vector<std::vector <double> > OccupancyGrid;

    class OccupancyState {
    public:
        enum {
            UNKNOWN = -1,
            FREE,
            OCCUPIED
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
}
