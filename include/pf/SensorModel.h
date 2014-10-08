#pragma once

#include <pf/Map.h>
#include <pf/RobotState.h>

namespace pf {
    class SensorModel {
    public:
        SensorModel(MapPtr map);
        double calculateLogWeight(std::vector<double> ranges, RobotState state);
        double probMeasurementAtPose(double measurement, double bearing, RobotState state);
        double probGaussian(double measurement, double nominal_range);
        double probUniform();
        // snap ranges greater than max_range to max_range
        void filterRanges(std::vector<double>& ranges);
        std::vector<double> undersampleData(std::vector<double> data);
    private:
        // map can be set only once
        MapPtr map_;
        std::vector<double> bearings;
    };
};
