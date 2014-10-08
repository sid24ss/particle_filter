#pragma once
#include <vector>
#include <string>
#include <stdexcept>

#include <pf/Constants.h>

namespace pf {
    // Represents coordinates in the standard odometry frame
    struct OdometryReading {
        double x;
        double y;
        double theta;
        bool operator==(const OdometryReading& other);
    };

    class SensorReading {
    public:
        SensorReading() : is_laser(false) {}
        OdometryReading robot_in_odom;
        OdometryReading laser_in_odom;
        std::vector<double> scan_data;
        bool is_laser;
        double ts;

        // debug
        void print();
    };

    class Log {
    public:
        Log(std::string file_name);
        SensorReading getNextReading();
        bool isEmpty() { return iterator_ == sensor_data_.size(); }
        size_t totalReadings() { return total_readings_; }
    private:
        bool loadFromFile(std::string file_name);
        std::vector<SensorReading> sensor_data_;
        size_t iterator_;
        size_t total_readings_;
    };
}