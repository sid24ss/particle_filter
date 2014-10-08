#include <pf/Resampler.h>
#include <pf/Utilities.h>
#include <algorithm>

using namespace pf;

Resampler::Resampler()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_ = std::default_random_engine(seed);
}

VanillaResampler::VanillaResampler()
{
    distribution_ = std::uniform_real_distribution<double>(0.0, 1);
}

std::vector<size_t> VanillaResampler::resample(std::vector<double>& weights)
{
    // std::vector<double> weights(log_weights);
    // std::vector<double> weights(log_weights.size(), 0);
    // for (size_t i = 0; i < log_weights.size(); ++i)
    //     weights[i] = std::exp(log_weights[i]);
    normalizeData(weights);
    // printf("normalized weights\n");
    // std::for_each(weights.begin(), weights.end(), [](double val){
    //     printf("%f ", val);
    // });
    // printf("\n");
    std::vector<double> cumsum(weights.size() + 1, 0);
    for (size_t i = 0; i < weights.size(); i++) {
        cumsum[i+1] = cumsum[i] + weights[i];
    }
    // printf("cumsum\n");
    // std::for_each(cumsum.begin(), cumsum.end(), [](double val){
    //     printf("%f ", val);
    // });
    // printf("\n");
    std::vector<size_t> idx;
    for (size_t i = 0; i < weights.size(); i++){
        double sample = distribution_(generator_);
        // printf("drawn : %f\n", sample);
        // TODO : Do a binary search
        size_t j = 0;
        while(cumsum[j] <= sample)
            j++;
        idx.push_back(j-1);
    }
    // printf("idx\n");
    // std::for_each(idx.begin(), idx.end(), [](double val){
    //     printf("%f ", val);
    // });
    // printf("\n");
    return idx;
}

LowVarianceResampler::LowVarianceResampler()
{
    distribution_ = std::uniform_real_distribution<double>(0.0, 1);
}

std::vector<size_t> LowVarianceResampler::resample(std::vector<double>& weights)
{
    normalizeData(weights);
    std::vector<double> cumsum(weights.size() + 1, 0);
    for (size_t i = 0; i < weights.size(); i++) {
        cumsum[i+1] = cumsum[i] + weights[i];
    }
    // printf("cumsum\n");
    // std::for_each(cumsum.begin(), cumsum.end(), [](double val){
    //     printf("%f ", val);
    // });
    // printf("\n");
    double r = distribution_(generator_);
    std::vector<size_t> idx;
    for (size_t i = 0; i < weights.size(); i++){
        // double sample = r + static_cast<double>(i/weights.size());
        double sample = r + static_cast<double>(i);
        size_t j = 0;
        while(cumsum[j]*static_cast<double>(weights.size()) <= sample && j <weights.size())
            j++;
        if (j == weights.size())
            j = 1;
        idx.push_back(j-1);
    }
    // printf("idx\n");
    // std::for_each(idx.begin(), idx.end(), [](double val){
    //     printf("%f ", val);
    // });
    // printf("\n");
    return idx;
}