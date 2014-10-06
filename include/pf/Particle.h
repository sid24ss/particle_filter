#pragma once
#Include <iostream>
#include <vector>

namespace pf {
    class Particle {
    public:
        Particle(RobotState state);
        void propagate();
    private:
        RobotState state_;
        };
};
