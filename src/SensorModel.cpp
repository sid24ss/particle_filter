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
    double prob_gaussian = probGaussian(measurement, nominal_range);
    double prob_uniform = probUniform();
    prob = SensorModelParams::mix_factor*prob_gaussian +
            (1 - SensorModelParams::mix_factor)*prob_uniform;
    // printf("nominal_range : %f;\t measurement : %f;\t prob : %f\n", nominal_range, measurement,
        // prob);
    return prob;
}

double SensorModel::probGaussian(double measurement, double nominal_range)
{
    double mean = nominal_range;
    // double std = SensorModelParams::variance_scaling*nominal_range;
    // Abhijeet : 3 sigma is 2 cm
    // double std = 2.0/3;
    double std = 1.0;
    double prob;

    // if (mean < SensorModelParams::max_range) {
            boost::math::normal_distribution<> distribution(mean, std);
            double normalizer = 
                boost::math::cdf(distribution, SensorModelParams::max_range)
                - boost::math::cdf(distribution, SensorModelParams::min_range);
            prob = boost::math::pdf(distribution, measurement)/normalizer;
    // }
    // else {
    //     prob = 0;
    //     // if (measurement == SensorModelParams::max_range)
    //     //     prob = 1;
    //     // else
    //     //     prob = 0;
    // }
    return prob;
}

double SensorModel::probUniform()
{
    return 0.2;
    // return 1.0/(SensorModelParams::max_range - SensorModelParams::min_range);
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

