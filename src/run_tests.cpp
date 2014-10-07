#include <pf/Test.h>

using namespace std;
using namespace pf;

int main () {
    Test t = Test();
    // if (!t.testMap("maps/wean.dat"))
    //     printf("map test failed.\n");
    // if (!t.testRobotState(std::vector<double> {0,1,0.1}))
    //     printf("robot state test failed.\n");
    // if (!t.testLogReading("log/robotdata1.log"))
    //     printf("log reading failed\n");
    // if (!t.testLogGetter("log/robotdata1.log"))
    //  printf("log getting failed\n");
    /*
    RobotState state_1(std::vector<double> {200,200,0});
    OdometryReading odom_1;
    odom_1.x = 0; odom_1.y = 0; odom_1.theta = 0;
    OdometryReading odom_2;
    odom_2.x = 100; odom_2.y = 0; odom_2.theta = 0;
    if (!t.testMotionModel(state_1, odom_1, odom_2))
        printf("motion model test failed\n");
    */
    // t.testBoostDistributions();
    t.testRayTrace("maps/wean.dat");
    return 0;
}
