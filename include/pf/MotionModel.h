#pragma once
#include <iostream>
#include <random>
#include <vector>

#include <pf/RobotState.h>
#include <pf/Log.h>

namespace pf {
    class MotionModel {
    public:
        MotionModel();
        RobotState sampleNextState(const RobotState& state_1, const OdometryReading& odom_1, const OdometryReading& odom_2);
    private:
        std::default_random_engine generator_;
        std::normal_distribution<double> distribution_;
    };
};
        
        
