#include <pf/RobotState.h>
#include <pf/Constants.h>

using namespace pf;

RobotState::RobotState() : state_(3,0)
{
}

RobotState::RobotState(std::vector<double> state) : state_(state)
{
}

void RobotState::x(double x)
{
    state_[RobotDOF::X] = x;
}

double RobotState::x()
{
    return state_[RobotDOF::X];
}

void RobotState::y(double y)
{
    state_[RobotDOF::Y] = y;
}

double RobotState::y()
{
    return state_[RobotDOF::Y];
}

void RobotState::theta(double theta)
{
    state_[RobotDOF::THETA] = theta;
}

double RobotState::theta()
{
    return state_[RobotDOF::THETA];
}

void RobotState::state(std::vector<double> state)
{
    state_.assign(state.begin(),state.end());
}

std::vector<double> RobotState::state()
{
    return state_;
}
