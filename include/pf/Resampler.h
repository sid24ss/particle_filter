#pragma once

#include <vector>
#include <random>
#include <chrono>

namespace pf {
    class Resampler {
    public:
        Resampler();
        // returns indices
        virtual std::vector<size_t> resample(std::vector<double> weights) = 0;
    protected:
        std::default_random_engine generator_;
    };

    class VanillaResampler : public Resampler {
    public:
        VanillaResampler();
        virtual std::vector<size_t> resample(std::vector<double> weights);
    private:
        std::uniform_real_distribution<double> distribution_;
    };
} // pf