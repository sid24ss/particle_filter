#pragma once
#include <string>
#include <vector>

#include <pf/RobotState.h>
#include <pf/Log.h>
#include <pf/Map.h>
#include <pf/Visualizer.h>
#include <pf/Constants.h>
#include <pf/Utilities.h>
#include <pf/MotionModel.h>


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
        bool testRayTrace(std::string map_name);
    };
};
