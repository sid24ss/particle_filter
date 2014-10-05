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
    private:
        static std::string window_name_;
    };
}