#pragma once
#include <string>
#include <vector>

namespace pf{
    class Test {
    public:
        bool testMap(std::string file_name);
        bool testRobotState(std::vector<double> state);
        bool testATan2(double y, double x);
        bool testShortestAngularDistance(double th_1, double th_2);
        bool testLogReading(std::string file_name);
    };
};
