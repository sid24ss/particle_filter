#pragma once
#include <vector>
#include <string>

namespace pf {
    // Represents coordinates of the robot in the standard odometry frame
    class OdometryReading {
    public:
        double x;
        double y;
        double theta;
        double ts;
    };

    class LaserReading {
    public:
        OdometryReading robot_in_odom;
        OdometryReading laser_in_odom;
        std::vector<int> readings;
        double ts;
    };

    class Log {
    public:
        Log(std::string file_name);
    private:
        bool loadFromFile(std::string file_name);
        std::vector<OdometryReading> odom_readings_;
        std::vector<LaserReading> laser_readings_;
    };
}