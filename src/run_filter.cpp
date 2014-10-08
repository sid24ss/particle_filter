#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include <pf/ParticleFilter.h>
#include <pf/Map.h>

using namespace std;
using namespace pf;

int main(int argc, char** argv)
{
	MapPtr map(new Map("maps/wean.dat"));
	ParticleFilter filter(map, "log/robotdata1.log", 2000);
    filter.initialize();
	// filter.debugParticles();
	// filter.debugSensorModel();
    filter.updateBelief();
    cv::waitKey(0);
	return 0;
}
