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
        static constexpr double alpha_1 = 0.01;
        // assumes distance in cm and angle in rad!
        static constexpr double alpha_2  = 0.01;
        static constexpr double alpha_3 = 0.1;
        static constexpr double alpha_4 = 0.1;
    };

    namespace SensorModelParams {
        // const double variance_scaling = 0.20;
        const double max_range = 8163.0;
        const double min_range = 0.0;
        inline std::vector<double> getBearings() {
            std::vector<double> bearings(180,0);
            std::iota(bearings.begin(), bearings.end(), -90);
            std::for_each(bearings.begin(), bearings.end(), [](double& val){
                val = DEG2RAD(val);
            });
            return bearings;
        }
        const double skip = 5;

        const double HIT_SIGMA = 20;
        const double SHORT_NOISE_LAMBDA = 0.0005;
        const double ZHIT = 0.8; //0.8
        const double ZNOISE = 10; //0.2
        const double ZSHORT = 0.0; //0.095;
        const double ZMAX = 0.0; //0.005;
    }

    namespace MapParams {
        // TODO: tune these parameters
        // Note : The Map is occupied if the prob_ value is 0
        // unoccupied if prob_ value is 1
        const double WALL_TOL = 0.95;
        const double FREE_TOL = 0.001;
    }

    namespace SamplerParams {
        const double VARIANCE_THRESHOLD = 0e-08;
    }

    struct FilterParams{
        std::string map_file;
        std::string log_file;
        size_t num_particles;
        bool record_video;
    };
}
