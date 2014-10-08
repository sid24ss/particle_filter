#include <cmath>
#include <random>
#include <chrono>

#include <boost/progress.hpp>

#include <pf/ParticleFilter.h>
#include <pf/Log.h>

using namespace pf;

ParticleFilter::ParticleFilter(MapPtr map_ptr,
                               std::string log_file_name,
                               size_t num_particles) : 
    map_(map_ptr),
    log_(log_file_name),
    particles_(num_particles),
    weights_(num_particles, 0),
    log_weights_(num_particles, 0),
    num_particles_(num_particles),
    motion_model_(map_),
    sensor_model_(map_),
    resampler_(new VanillaResampler()),
    viz_("particle_filter", map_ptr)
{ }

void ParticleFilter::initialize()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::pair <double, double> map_size = map_->getSize();
    std::uniform_real_distribution<double> x_distribution(0.0, map_size.first);
    std::uniform_real_distribution<double> y_distribution(0.0, map_size.second);
    std::uniform_real_distribution<double> theta_distribution(-M_PI, M_PI);

    size_t count = 0;
    double x, y, theta;
    while (count < num_particles_) {
        x = x_distribution(generator);
        y = y_distribution(generator);
        theta = theta_distribution(generator);
        if (map_->isFree(x, y)) {
            particles_[count] = RobotState(x, y, theta);
            count = count+1;
        }
    }
}

void ParticleFilter::updateBelief() {
    OdometryReading odom_previous;
    OdometryReading odom_current;
    SensorReading current_reading;

    bool first_time = true;
    boost::progress_display show_progress(log_.totalReadings());
    visualizeParticles();
    while (!log_.isEmpty()) {
    // while we still have sensor readings
        // get the next reading
        current_reading = log_.getNextReading();
        // we always need the odometry
        odom_current = current_reading.robot_in_odom;

        // first time we run, we have no previous odometry measurements.
        // we must propagate.
        if (first_time) {
            odom_previous = odom_current;
            first_time = false;
            if (!current_reading.is_laser)
                continue;
        }

        propagate(odom_previous, odom_current);
        // visualizeParticles();
        if (current_reading.is_laser) {
            calculateW(current_reading.scan_data);
            resample();
        }        
        odom_previous = odom_current;
        ++show_progress;
        printf("waiting for keyboard input\n");
        // std::cin.get();
        visualizeParticles();
        if (current_reading.is_laser){
            // find most likely state
            size_t max_idx = std::max_element(log_weights_.begin(),
                log_weights_.end()) - log_weights_.begin();
            viz_.visualizeScan(particles_[max_idx],
                current_reading.scan_data);
        }
    }
}

void ParticleFilter::debugParticles() {
    int i = 0;
    for (auto& particle : particles_) {
        printf("current particle (%d) : \n", i);
        particle.print();
        assert(map_->isFree(particle.x(), particle.y()));
        i++;
    }
}

void ParticleFilter::debugSensorModel() {
    SensorReading reading;
    while (true) {
        reading = log_.getNextReading();
        if(reading.is_laser)
            break;
    }
    calculateW(reading.scan_data);
    resample();
}

/**
 * @brief propagates only the motion model
 * @details Irrespective of Odom or Laser reading, we need to sample from the 
 * motion model, and then (in case of laser), update using sensor model
 * 
 * @param odom_p the previous odometry reading
 * @param odom_n the next odometry reading
 */
void ParticleFilter::propagate(OdometryReading odom_p, OdometryReading odom_n)
{
    // return if previous reading and the current reading are the same
    if (odom_p == odom_n)
        return;
    // if not, we need to invoke the motion model and move all particles
    for (auto& particle : particles_) {
        particle = motion_model_.sampleNextState(particle, odom_p, odom_n);
    }
}

/**
 * @brief calculates the weights for this iteration
 * 
 * @param ranges : scan data from the sensor reading
 */
void ParticleFilter::calculateW(std::vector<double> scan_data)
{
    for (size_t i = 0; i < particles_.size(); ++i) {
        // printf("i: %d, weight: %.4f\n", i, weights_[i]);
        log_weights_[i] = sensor_model_.calculateLogWeight(scan_data, particles_[i]);
        // weights_[i] = sensor_model_.calculateWeight(scan_data, particles_[i]);
    }
}

void ParticleFilter::resample()
{
    std::vector<size_t> idx = std::move(resampler_->resample(log_weights_));
    std::vector<RobotState> new_particles;
    std::for_each(idx.begin(), idx.end(),
        [&new_particles, &particles_](int id){
            new_particles.emplace_back(particles_[id]);
        }
    );
    particles_ = std::move(new_particles);
}

void ParticleFilter::visualizeParticles()
{
    viz_.visualizePoses(particles_);
}
