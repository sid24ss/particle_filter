#include <pf/RobotState.h>

using namespace pf;

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
