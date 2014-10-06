#include <pf/RobotState.h>

using namespace pf;

void RobotState::state(std::vector<double> state)
{
    state_.assign(state.begin(),state.end());
}
