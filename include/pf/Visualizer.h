#pragma once
#include <iostream>
#include <string>
#include <vector>

#include <pf/Constants.h>

namespace pf {
    class Visualizer {
    public:
        static void setWindowName(std::string name) { window_name_ = name; }
        static void visualizeArray(OccupancyGrid& grid);
        // static void visualizeVector(size_t x, size_t y, const std::vector<double>& data);
    private:
        static std::string window_name_;
    };
}