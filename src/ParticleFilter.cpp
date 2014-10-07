#include <cmath>
#include <random>
#include <chrono>

#include <pf/ParticleFilter.h>
#include <pf/Log.h>

using namespace pf;

void ParticleFilter::initialize()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::pair <double, double> map_size = map_->getSize();
    std::uniform_real_distribution<double> x_distribution(0.0, map_size.first);
    std::uniform_real_distribution<double> y_distribution(0.0, map_size.second);
    std::uniform_real_distribution<double> theta_distribution(-M_PI, M_PI);

    int count = 0;
    double x, y, theta;
    while (count < num_particles_) {
        x = x_distribution(generator);
        y = y_distribution(generator);
        theta = theta_distribution(generator);
        if (map_->isFree(x, y)) {
            particles_.push_back(RobotState(x, y, theta));
            count = count+1;
        }
    }
}

void ParticleFilter::updateBelief() {
    OdometryReading odom_previous;
    OdometryReading odom_current;
    SensorReading current_reading;

    bool first_time = true;
    while (~log_.isEmpty()) {
        current_reading = log_.getNextReading();
        odom_current = current_reading.robot_in_odom;
        
        if (first_time) {
            odom_previous = odom_current;
            first_time = false;
            continue;
        }        

        propagate(odom_previous, odom_current);
        if (current_reading.is_laser) {
            calculateW(current_reading.scan_data);
            resample();
        }        
        odom_previous = odom_current;
    }

}

void ParticleFilter::propagate(OdometryReading odom_1, OdometryReading odom_2)
{
}

void ParticleFilter::resample()
{
}

void ParticleFilter::calculateW(std::vector<double> ranges)
{
    for (size_t i = 0; i < particles_.size(); ++i) {
        weights_[i] = sensor_model_.calculateWeight(ranges, particles_[i]);
    }
}
