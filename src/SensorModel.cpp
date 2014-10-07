#include <boost/math/distributions.hpp>

#include <pf/Constants.h>
#include <pf/SensorModel.h>

using namespace pf;

SensorModel::SensorModel(Map map) : 
    map_(map),
    K_(0.01),
    max_range_(800.0),
    min_range_(0.0)
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
    double std = K_*nominal_range;
    double prob;
    
    if (mean < max_range_)
        {
            // TODO: is direct computation faster than using Boost distributions?
            boost::math::normal_distribution<> distribution(mean, std);
            if(measurement < min_range_ || measurement > max_range_)
                prob = 0;
            else
                {
                    double normalizer = boost::math::cdf(distribution, max_range_) - boost::math::cdf(distribution, min_range_);
                    prob = boost::math::pdf(distribution, measurement)/normalizer;
                }
        }
    else
        {
            // TODO: what is this behavior?
        }
    return prob;
}
