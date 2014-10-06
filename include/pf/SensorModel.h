#pragma once

#include <pf/Map.h>

namespace pf {
    class SensorModel {
    public:
        SensorModel(Map map);
        double probMeasurementAtPose(double measurement, RobotState state);
        double probMeasurement(double measurement, doule nominal_range);
    private:
        // map can be set only once
        Map map_
            };
};
