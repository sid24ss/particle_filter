#pragma once

#include <pf/Map.h>
#include <pf/RobotState.h>

namespace pf {
    class SensorModel {
    public:
        SensorModel(Map map);
        double probMeasurementAtPose(double measurement, RobotState state);
        double probMeasurement(double measurement, double nominal_range);
    private:
        // map can be set only once
        Map map_;
        double K_;
        double max_range_;
        double min_range_;
    };
};
