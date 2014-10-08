#include <pf/RobotState.h>

using namespace pf;

RobotState::RobotState(const RobotState& other)
    : state_(3,0)
{
    x(other.x());
    y(other.y());
    theta(other.theta());
}

RobotState::RobotState(std::vector<double> state)
    : state_(3, 0)
{
    x(state[RobotDOF::X]);
    y(state[RobotDOF::Y]);
    theta(state[RobotDOF::THETA]);
}


RobotState::RobotState(double _x, double _y, double _theta)
    : state_(3,0)
{
    x(_x);
    y(_y);
    theta(_theta);
}

void RobotState::state(std::vector<double> state)
{
    state_.assign(state.begin(),state.end());
}

std::vector<double> RobotState::getLaserCoords() const
{
    std::vector<double> laser_coords(state_.size(), 0);
    laser_coords[RobotDOF::X] = state_[RobotDOF::X] + 25*std::cos(state_[RobotDOF::THETA]);
    laser_coords[RobotDOF::Y] = state_[RobotDOF::Y] + 25*std::sin(state_[RobotDOF::THETA]);
    laser_coords[RobotDOF::THETA] = state_[RobotDOF::THETA];
    return laser_coords;
}

void RobotState::print()
{
    printf("\t\tx: %f\ty: %f\ttheta: %f\n", x(), y(), theta());
}

