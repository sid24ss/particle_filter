#include <cmath>
#include <random>
#include <limits>
#include <chrono>
// #include <omp.h>

#include <boost/progress.hpp>
#include <boost/bind.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <pf/ParticleFilter.h>
#include <pf/Log.h>

using namespace pf;
using namespace boost;

ParticleFilter::ParticleFilter(FilterParams params) : 
    params_(params),
    map_(new Map(params.map_file)),
    log_(params.log_file),
    particles_(params.num_particles),
    weights_(params.num_particles, 0),
    log_weights_(params.num_particles, 0),
    num_particles_(params.num_particles),
    motion_model_(map_),
    sensor_model_(map_),
    resampler_(new VanillaResampler()),
    viz_("particle_filter", map_),
    record_video_(params.record_video)
{
    video_writer_.open(params.video_file_name, params.fourcc, params.fps, params.
        frame_size, true);
}

void ParticleFilter::initialize()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::pair <double, double> map_size = map_->getSize();
    std::uniform_real_distribution<double> x_distribution(0.0, map_size.first);
    std::uniform_real_distribution<double> y_distribution(0.0, map_size.second);
    // std::uniform_real_distribution<double> x_distribution(3650, 4250);
    // std::uniform_real_distribution<double> y_distribution(4200, 4840);
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
    bool has_moved = false;
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

        // visualizeParticles();
        if (current_reading.is_laser) {
            calculateW(current_reading.scan_data);
            resample();
            // Scan visualization
            // find most likely state
            size_t max_idx = std::max_element(log_weights_.begin(),
                log_weights_.end()) - log_weights_.begin();
            viz_.plotRayTrace(particles_[max_idx], SensorModel::undersampleData(SensorModelParams::getBearings()));
            viz_.visualizeScan(particles_[max_idx], current_reading.scan_data);
        }
        has_moved = propagate(odom_previous, odom_current);
        
        odom_previous = odom_current;
        ++show_progress;
        visualizeParticles();
        // printf("waiting for keyboard input\n");
        // std::cin.get();
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
    for (auto log_weight : log_weights_) {
      printf("%.3f ", std::exp(log_weight));
    }
    printf("\n");
    resample();
    for (auto weight : weights_) {
      printf("%.3f ", weight);
    }
    printf("\n");
    visualizeParticles();
}

/**
 * @brief propagates only the motion model
 * @details Irrespective of Odom or Laser reading, we need to sample from the 
 * motion model, and then (in case of laser), update using sensor model
 * 
 * @param odom_p the previous odometry reading
 * @param odom_n the next odometry reading
 */
bool ParticleFilter::propagate(OdometryReading odom_p, OdometryReading odom_n)
{
    // return if previous reading and the current reading are the same
    if (odom_p == odom_n)
        return false;
    // if not, we need to invoke the motion model and move all particles
    // #pragma omp parallel for
    // for (auto& particle : particles_) {
    //     particle = motion_model_.sampleNextState(particle, odom_p, odom_n);
    // }

    // #pragma omp parallel for
    for (size_t i = 0; i < particles_.size(); ++i) {
        particles_[i] = motion_model_.sampleNextState(particles_[i], odom_p, odom_n);
    }
    return true;
}

/**
 * @brief calculates the weights for this iteration
 * 
 * @param ranges : scan data from the sensor reading
 */
void ParticleFilter::calculateW(std::vector<double> scan_data)
{
     //#pragma omp threadprivate(particles_)
    // int th_id;
    // #pragma omp parallel for
    for (size_t i = 0; i < particles_.size(); ++i) {
        // th_id = omp_get_thread_num();
     // printf("thread number %d\n", th_id);
        // printf("i: %d, weight: %.4f\n", i, weights_[i]);
        // if (!map_->isFree(particles_[i].x(), particles_[i].y()))
        //     log_weights_[i] = - std::numeric_limits<double>::infinity();
        // else
            log_weights_[i] = sensor_model_.calculateLogWeight(scan_data, particles_[i]);        
    }
}

void ParticleFilter::resample()
{
    weights_.resize(log_weights_.size());
    for (size_t i = 0; i < log_weights_.size(); ++i) {
        weights_[i] = std::exp(log_weights_[i]);
        weights_[i] = std::pow(weights_[i],SamplerParams::POWER_SCALE);
    }
    if (compute_particle_variance() >= SamplerParams::VARIANCE_THRESHOLD){
        std::vector<size_t> idx = std::move(resampler_->resample(weights_));
        std::vector<RobotState> new_particles;
        std::for_each(idx.begin(), idx.end(),
            [&new_particles, &particles_](int id){
                new_particles.emplace_back(particles_[id]);
            }
        );
        particles_ = std::move(new_particles);
    } else {
        printf("Weight variance log: %f\n", std::log(compute_particle_variance()));
        // do nothing
    }
}

void ParticleFilter::visualizeParticles()
{
    viz_.visualizePoses(particles_);
    if (record_video_) {
        video_writer_.write(viz_.getCurrentImage()); 
    }
}

double ParticleFilter::compute_particle_variance()
{
    // compute the variance over the weights
    accumulators::accumulator_set<double, accumulators:: stats<accumulators::tag::variance> > acc;
    for_each(weights_.begin(), weights_.end(), bind<void>(ref(acc), _1));

    // cout << mean(acc) << endl;
    // cout << (variance(acc)) << endl;
    return accumulators::variance(acc);
}
