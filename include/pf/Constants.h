#pragma once

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
        enum{
            X,
            Y,
            THETA
        };
    };
}