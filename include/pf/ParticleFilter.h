#pragma once
#include <vector>
#include <string>
#include <memory>

#include <pf/MotionModel.h>
#include <pf/SensorModel.h>
#include <pf/Map.h>
#include <pf/Particle.h>
#include <pf/Log.h>

namespace pf {
    class ParticleFilter
    {
    public:
        ParticleFilter(MapPtr map_ptr, std::string file_name, size_t num_particles) : map_(map_ptr), log_(file_name), num_particles_(num_particles), motion_model_(), sensor_model_(map_) {}
        ~ParticleFilter();
        void initialize();
        void updateBelief();
        void propagate(OdometryReading odom_1, OdometryReading odom_2);
        void calculateW(std::vector<double> data);
        void resample();
    private:
        MapPtr map_;
        Log log_;
        std::vector<RobotState> particles_;
        std::vector<double> weights_;
        size_t num_particles_;
        MotionModel motion_model_;
        SensorModel sensor_model_;
    };
}
