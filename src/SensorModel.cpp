#include <cmath>
#include <boost/math/distributions.hpp>

#include <pf/Constants.h>
#include <pf/SensorModel.h>

using namespace pf;

SensorModel::SensorModel(MapPtr mapptr) :
    map_(mapptr)
{}

double SensorModel::calculateLogWeight(std::vector<double> ranges, RobotState state)
{
    // filterRanges(ranges);
    ranges = undersampleData(ranges);
    bearings = SensorModelParams::getBearings();
    bearings = undersampleData(bearings);
    assert(ranges.size() == bearings.size());
    // double prob = 1;
    double logprob = 0;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        logprob += std::log(probMeasurementAtPose(ranges[i], bearings[i], state));
        // prob = prob*probMeasurementAtPose(ranges[i], bearings[i], state);
    }
    // printf("logprob : %f\n", logprob);
    return logprob;
}

double SensorModel::probMeasurementAtPose(double measurement, double bearing, RobotState state)
{
    // TODO: get nominal range by raycasting
    double nominal_range = map_->getNominalReading(state, bearing);
    double prob;
    prob = SensorModelParams::ZHIT  *   probGaussian(measurement, nominal_range)
        +  SensorModelParams::ZNOISE*   probUniform()
        +  SensorModelParams::ZSHORT*   probDecaying(measurement, nominal_range)
        +  SensorModelParams::ZMAX  *   probMaxNoise(measurement);
    // printf("nominal_range : %f;\t measurement : %f;\t prob : %f\n", nominal_range, measurement,
        // prob);
    assert(prob != 0.0);
    return prob;
}

double SensorModel::probGaussian(double measurement, double nominal_range)
{
    double mean = nominal_range;
    double prob;

    boost::math::normal_distribution<> distribution(mean, SensorModelParams::HIT_SIGMA);
    prob = boost::math::pdf(distribution, measurement);

    return prob;
}

double SensorModel::probUniform()
{
    // return 0.2;
    return 1.0/(SensorModelParams::max_range - SensorModelParams::min_range);
}

double SensorModel::probMaxNoise(double measurement)
{
    return static_cast<double>(measurement == SensorModelParams::max_range);
}

double SensorModel::probDecaying(double measurement, double nominal_range)
{
    double res = 0;
    double norm = 1/(1-std::exp(-SensorModelParams::SHORT_NOISE_LAMBDA*nominal_range));
    if (measurement < nominal_range) {
        res = SensorModelParams::SHORT_NOISE_LAMBDA*std::exp(-SensorModelParams::SHORT_NOISE_LAMBDA*measurement)*norm;
    }
    return res;
}

void SensorModel::filterRanges(std::vector<double>& ranges)
{
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] > SensorModelParams::max_range)
            ranges[i] = SensorModelParams::max_range;
    }
}

std::vector<double> SensorModel::undersampleData(std::vector<double> data)
{
    std::vector<double> undersampled_data;
    for (std::size_t i =0; i < data.size(); i = i + SensorModelParams::skip) {
        undersampled_data.push_back(data[i]);
    }
    return undersampled_data;
}

