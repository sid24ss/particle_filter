#pragma once
#include <iostream>
#include <vector>

#include <pf/RobotState.h>

namespace pf {
    class Particle {
    public:
        Particle(RobotState state);
        void propagate();
        double weight() const { return weight_; }
        void weight(double w) { weight_ = w; }

    private:
        double weight_;
        RobotState state_;
    };
};
