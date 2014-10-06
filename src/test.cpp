#include <pf/Test.h>

using namespace std;
using namespace pf;

int main () {
    Test t = Test();
    //t.testMap("maps/wean.dat");
    t.testRobotState(std::vector<double> {0,1,0.1});
    return 0;
}
