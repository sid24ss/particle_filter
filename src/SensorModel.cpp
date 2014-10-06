#include <pf/SensorModel.h>

using namespace pf;

SensorModel::SensorModel(Map map) : 
    map_(map) {}

double SensorModel::probMeasurementAtPose(double measurement, RobotState state)
{
    // TODO: get nominal range
    double nominal_range = 0.5; 
    // use constants
    double prob = probMeasurement(measurement, nominal_range);
    return prob;
}

