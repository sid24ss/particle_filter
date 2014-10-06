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
    if (!t.testLogGetter("log/robotdata1.log"))
        printf("log getting failed\n");
    return 0;
}
