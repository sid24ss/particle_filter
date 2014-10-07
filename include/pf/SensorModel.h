#pragma once

#include <pf/Map.h>
#include <pf/RobotState.h>

namespace pf {
    class SensorModel {
    public:
        SensorModel(MapPtr map);
        double probMeasurementAtPose(double measurement, RobotState state);
        double probMeasurement(double measurement, double nominal_range);
    private:
        // map can be set only once
        MapPtr map_;
        // double variance_scaling_;
        // double max_range_;
        // double min_range_;
    };
};
