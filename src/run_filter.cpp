#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include <pf/ParticleFilter.h>
#include <pf/Map.h>

using namespace std;
using namespace pf;

int main(int argc, char** argv)
{
	// MapPtr map(new Map("maps/wean.dat"));
    FilterParams params;
    params.map_file = std::string("maps/wean.dat");
    params.log_file = std::string("log/robotdata1.log");
    params.num_particles = 3000;
    params.record_video = false;
	ParticleFilter filter(params);
    filter.initialize();
    filter.visualizeParticles();
    std::cin.get();
	// filter.debugParticles();
	// filter.debugSensorModel();
    filter.updateBelief();
    cv::waitKey(0);
	return 0;
}
