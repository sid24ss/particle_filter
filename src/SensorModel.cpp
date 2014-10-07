#include <boost/math/distributions.hpp>

#include <pf/Constants.h>
#include <pf/SensorModel.h>

using namespace pf;

SensorModel::SensorModel(MapPtr mapptr) :
    map_(mapptr)
{}

double SensorModel::calculateWeight(std::vector<double> ranges, RobotState state)
{
    filterRanges(ranges);
    ranges = undersampleData(ranges);
    bearings = SensorModelParams::getBearings();
    bearings = undersampleData(bearings);
    double prob = 1;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        prob = prob*probMeasurementAtPose(ranges[i], bearings[i], state);
    }
    return prob;
}

double SensorModel::probMeasurementAtPose(double measurement, double bearing, RobotState state)
{
    // TODO: get nominal range by raycasting
    double nominal_range = map_->getNominalReading(state, bearing);
    double prob = probMeasurement(measurement, nominal_range);
    return prob;
}

double SensorModel::probMeasurement(double measurement, double nominal_range)
{
    double mean = nominal_range;
    double std = SensorModelParams::variance_scaling*nominal_range;
    double prob;

    if (mean < SensorModelParams::max_range) {
            boost::math::normal_distribution<> distribution(mean, std);
            double normalizer = 
                boost::math::cdf(distribution, SensorModelParams::max_range)
                - boost::math::cdf(distribution, SensorModelParams::min_range);
            prob = boost::math::pdf(distribution, measurement)/normalizer;
    }
    else {
        if (measurement == SensorModelParams::max_range)
            prob = 1;
        else
            prob = 0;
    }
    return prob;
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

