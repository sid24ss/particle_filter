#include <boost/math/distributions.hpp>

#include <pf/Constants.h>
#include <pf/SensorModel.h>

using namespace pf;


SensorModel::SensorModel(MapPtr mapptr) :
    map_(mapptr)
    // variance_scaling_(0.01),
    // max_range_(800.0),
    // min_range(0.0)
{}

double SensorModel::probMeasurementAtPose(double measurement, RobotState state)
{
    // TODO: get nominal range by raycasting
    double nominal_range = 0.5; 
    double prob = probMeasurement(measurement, nominal_range);
    return prob;
}

double SensorModel::probMeasurement(double measurement, double nominal_range)
{
    double mean = nominal_range;
    double std = SensorModelParams::variance_scaling*nominal_range;
    double prob;
    
    if (mean < SensorModelParams::max_range) {
        // TODO: is direct computation faster than using Boost distributions?
        boost::math::normal_distribution<> distribution(mean, std);
        if(measurement < SensorModelParams::min_range || measurement > SensorModelParams::max_range) {
            prob = 0;
        } else {
            double normalizer = 
                boost::math::cdf(distribution, SensorModelParams::max_range)
              - boost::math::cdf(distribution, SensorModelParams::min_range);
            prob = boost::math::pdf(distribution, measurement)/normalizer;
        }
    } else {
        // TODO: what is this behavior?
    }
    return prob;
}
