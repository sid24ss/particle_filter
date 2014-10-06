#include <string>
#include <vector>
#include <math.h>

#include <pf/Constants.h>
#include <pf/Utilities.h>
#include <pf/Test.h>
#include <pf/Map.h>
#include <pf/RobotState.h>

using namespace pf;

Test::Test()
{
}

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

bool Test::testATan2(double y, double x)
{
    double th = atan2(y, x);
    printf("Answer in degrees: %.2f.\n", RAD2DEG(th));
}

bool Test::testShortestAngularDistance(double th_1, double th_2)
{
    printf("Answer in degrees: %.2f.\n",RAD2DEG(shortest_angular_distance(DEG2RAD(th_1),DEG2RAD(th_2))));
}
