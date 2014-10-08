#pragma once

#include <vector>
#include <random>
#include <chrono>

namespace pf {
    class Resampler {
    public:
        Resampler();
        // returns indices
        virtual std::vector<size_t> resample(std::vector<double>& weights) = 0;
        virtual void setNumParticles(size_t num_particles) {num_particles_ = num_particles;}
    protected:
        std::default_random_engine generator_;
        size_t num_particles_;
    };

    class VanillaResampler : public Resampler {
    public:
        VanillaResampler();
        virtual std::vector<size_t> resample(std::vector<double>& weights);
    private:
        std::uniform_real_distribution<double> distribution_;
    };

    class LowVarianceResampler : public Resampler {
    public:
        LowVarianceResampler();
        virtual std::vector<size_t> resample(std::vector<double>& weights);
    private:
        std::uniform_real_distribution<double> distribution_;
    };
} // pf