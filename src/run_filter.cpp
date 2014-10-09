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
    params.num_particles = 20000;
    params.record_video = true;
    params.video_file_name = "video_log_1.avi";
    params.fourcc = CV_FOURCC('m','p','4','v');
    params.fps = 18;
    params.frame_size = cv::Size(800,800);
	ParticleFilter filter(params);
    filter.initialize();
    filter.visualizeParticles();
    //std::cin.get();
	// filter.debugParticles();
    // filter.debugSensorModel();
    filter.updateBelief();
    cv::waitKey(0);
	return 0;
}
