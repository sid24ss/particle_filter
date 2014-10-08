#pragma once
#include <iostream>
#include <vector>
#include <string>

#include <pf/Constants.h>

namespace pf {
    class RobotState {
    public:
        RobotState() : state_(3, 0) { }
        RobotState(const RobotState& other);
        RobotState(std::vector<double> state);
        RobotState(double x, double y, double theta);
        // set and get methods for x
        void x(double x) { state_[RobotDOF::X] = x; }
        double x() const { return state_[RobotDOF::X]; }
        // set and get methods for y
        void y(double y) { state_[RobotDOF::Y] = y; }
        double y() const { return state_[RobotDOF::Y]; }
        // set and get methods for theta
        void theta(double theta) { state_[RobotDOF::THETA] = theta; }
        double theta() const { return state_[RobotDOF::THETA]; }
        // set and get methods for pose
        void state(std::vector<double> state);
        std::vector<double> state() const { return state_; }

        std::vector<double> getLaserCoords() const;

        void print();
    private:
        std::vector<double> state_;
    };
};
