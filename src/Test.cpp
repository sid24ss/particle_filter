#include <string>
#include <vector>
#include <cmath>

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

bool Test::testLogGetter(std::string file_name)
{
    Log log_data(file_name);
    // read the first test_num values
    int test_num = 10;
    for (int i = 0; i < test_num; ++i) {
        SensorReading next_reading = log_data.getNextReading();
        if (!next_reading.is_laser) {
            printf("ODOM type\n");
            next_reading.print();
        } else {
            printf("LASER type\n");
            next_reading.print();
        }
    }
    return true;
}