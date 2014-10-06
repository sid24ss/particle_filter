#include <pf/RobotState.h>

using namespace pf;

RobotState::RobotState()
{
    std::vector<double> state_ (3,0);
}

RobotState::RobotState(std::vector<double> state)
{
    std::vector<double> state_(state);
}

void RobotState::x(double x)
{
    state_[0] = x;
}

double RobotState::x()
{
    return state_[0];
}

void RobotState::y(double y)
{
    state_[1] = y;
}

double RobotState::y()
{
    return state_[1];
}

void RobotState::theta(double theta)
{
    state_[2] = theta;
}

double RobotState::theta()
{
    return state_[2];
}

void RobotState::state(std::vector<double> state)
{
    state_.assign(state.begin(),state.end());
}

std::vector<double> RobotState::state()
{
    return state_;
}
