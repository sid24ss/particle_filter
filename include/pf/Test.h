#pragma once
#include <string>
#include <vector>

namespace pf{
    class Test {
    public:
        bool testMap(std::string file_name);
        bool testRobotState(std::vector<double> state);
        bool testLogReading(std::string file_name);
    };
};
