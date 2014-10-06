#include <string>
#include <vector>

#include <pf/RobotState.h>
#include <pf/Constants.h>
#include <pf/Test.h>
#include <pf/Map.h>
#include <pf/Log.h>

using namespace pf;

bool Test::testMap(std::string file_name)
{
    Map map;
    if (map.loadFromFile(file_name))
        {
            printf("Successfully loaded the map.\n");
            map.visualize();
            return true;
        }
    else
        {
            printf("Oops. Map wasn't read.\n");
            return false;
        }
}

bool Test::testRobotState(std::vector<double> input_state)
{
    RobotState state(input_state);
    printf("x: %.2f, y: %.2f; theta: %.2f\n", state.x(), state.y(), state.theta());
    state.x(-1); state.y(-1); state.theta(M_PI);
    printf("x: %.2f, y: %.2f; theta: %.2f\n", state.x(), state.y(), state.theta());
    return true;
}

bool Test::testLogReading(std::string file_name)
{
    Log log_reader(file_name);
    return true;
}
