#pragma once
#include <string>
#include <vector>

namespace pf{
    class Test {
    public:
        Test();
        bool testMap(std::string file_name);
        bool testRobotState(std::vector<double> state);
    };
};
