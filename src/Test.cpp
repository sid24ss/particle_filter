#include <string>
#include <vector>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pf/RobotState.h>
#include <pf/Constants.h>
#include <pf/Utilities.h>
#include <pf/Test.h>
#include <pf/Map.h>
#include <pf/Log.h>
#include <pf/MotionModel.h>

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

bool Test::testATan2(double y, double x)
{
    double th = atan2(y, x);
    printf("Answer in degrees: %.2f.\n", RAD2DEG(th));
    return true;
}

bool Test::testShortestAngularDistance(double th_1, double th_2)
{
    printf("Answer in degrees: %.2f.\n",RAD2DEG(shortest_angular_distance(DEG2RAD(th_1),DEG2RAD(th_2))));
    return true;
}

bool Test::testLogReading(std::string file_name)
{
    Log log_reader(file_name);
    return true;
}

bool Test::testMotionModel(RobotState state_1, OdometryReading odom_1, OdometryReading odom_2)
{
    MotionModel model = MotionModel();
    cv::Mat image =  cv::Mat::zeros( 400, 400, CV_8UC3 );
    cv::circle(image, cv::Point(state_1.x(), state_1.y()), 0.5, cv::Scalar(0, 0, 255), -1);
    return true;
}
