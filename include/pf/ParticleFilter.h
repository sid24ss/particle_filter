#pragma once
#include <vector>
#include <string>
#include <memory>

#include <pf/Map.h>
#include <pf/Particle.h>
#include <pf/Log.h>

namespace pf {
    class ParticleFilter
    {
    public:
        ParticleFilter(MapPtr map_ptr) : map_(map_ptr) {}
        ~ParticleFilter();
        void propagate();
        void calculateW();
    private:
        MapPtr map_;
        Log log_;
        std::vector<Particle> particles_;
        size_t num_particles_;
    };
}