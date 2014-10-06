#pragma once
#include <iostream>
#include <vector>

#include <pf/RobotState.h>

namespace pf {
    class Particle {
    public:
        Particle(RobotState state);
        void propagate();
    private:
        RobotState state_;
        };
};
