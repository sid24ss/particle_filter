#pragma once
#include <iostream>
#include <vector>
#include <string>

#include <pf/Constants.h>

namespace pf{
    class RobotState {
    public:
        RobotState();
        RobotState(std::vector<double> state);
        // set and get methods for x
        void x(double x);
        double x();
        // set and get methods for y
        void y(double y);
        double y();
        // set and get methods for theta
        void theta(double theta);
        double theta();
        // set and get methods for pose
        void state(std::vector<double> state);
        std::vector<double> state();
    private:
        std::vector<double> state_;
    };
};
