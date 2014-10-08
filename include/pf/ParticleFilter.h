#pragma once
#include <vector>
#include <string>
#include <memory>

#include <pf/MotionModel.h>
#include <pf/SensorModel.h>
#include <pf/Resampler.h>
#include <pf/Map.h>
#include <pf/Particle.h>
#include <pf/Log.h>
#include <pf/Visualizer.h>

namespace pf {
    class ParticleFilter
    {
    public:
        ParticleFilter(MapPtr map_ptr, std::string file_name, size_t num_particles);
        void initialize();
        void updateBelief();
        void propagate(OdometryReading odom_1, OdometryReading odom_2);
        void calculateW(std::vector<double> data);
        void resample();
        void visualizeParticles();
        void debugSensorModel();
        void debugParticles();
    private:
        MapPtr map_;
        Log log_;
        std::vector<RobotState> particles_;
        std::vector<double> weights_;
        std::vector<double> log_weights_;
        size_t num_particles_;
        MotionModel motion_model_;
        SensorModel sensor_model_;
        std::unique_ptr<Resampler> resampler_;
        Visualizer viz_;
    };
}
