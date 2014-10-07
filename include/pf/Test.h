#pragma once
#include <string>
#include <vector>

#include <pf/RobotState.h>
#include <pf/Log.h>

namespace pf{
    class Test {
    public:
        bool testMap(std::string file_name);
        bool testRobotState(std::vector<double> state);
        bool testATan2(double y, double x);
        bool testShortestAngularDistance(double th_1, double th_2);
        bool testLogReading(std::string file_name);
        bool testMotionModel(RobotState state_1, OdometryReading odom_1, OdometryReading odom_2);
        bool testLogGetter(std::string file_name);
        bool testBoostDistributions();
    };
};
