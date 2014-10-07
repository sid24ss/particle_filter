#include <string>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/math/distributions.hpp>

#include <pf/Test.h>

using namespace pf;

bool Test::testMap(std::string file_name)
{
    Map map;
    if (map.loadFromFile(file_name))
        {
            printf("Successfully loaded the map.\n");
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
    RobotState state_2;
    MotionModel model = MotionModel();
    cv::Mat image =  cv::Mat::zeros( 400, 400, CV_8UC3 );
    cv::circle(image, cv::Point(state_1.x(), state_1.y()), 2, cv::Scalar(0, 0, 255), -1, 8);
    for (int i = 0; i < 50; ++i) {
        state_2 = model.sampleNextState(state_1, odom_1, odom_2);
        cv::circle(image, cv::Point(state_2.x(), state_2.y()), 2, cv::Scalar(255, 0, 0), -1, 8);
        }
    cv::imshow("Image",image);
    cv::waitKey(0);
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

bool Test::testBoostDistributions()
{
    double mean = 0;
    double std = 1;
    double query = 0.5;
    boost::math::normal_distribution<> pdf(mean, std);
    printf("Distribution with mean: %.2f and std: %.2f.\n", mean, std);
    printf("cdf at %.2f: %.6f.\n", query, boost::math::cdf(pdf, query));
    printf("pdf at %.2f: %.6f.\n", query, boost::math::pdf(pdf, query));
    return true;
}

bool Test::testRayTrace(std::string map_file)
{
    MapPtr mapptr(new Map());
    bool map_loaded = mapptr->loadFromFile(map_file);
    if (!map_loaded)
        return false;
    Visualizer viz("test_ray_trace", mapptr);
    RobotState test_state(400, 400, 0);
    // viz.showMap();
    viz.plotRayTrace(test_state);
    cv::waitKey(0);
    return true;
}