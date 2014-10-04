#pragma once
#include <iostream>
#include <vector>
#include <string>

#include <pf/Constants.h>

namespace pf {
    /**
     * @brief contains the map
     */
    class Map {
    public:
        Map();
        // returns the dimensions of the cells underneath.
        std::pair <size_t, size_t> getDims();
        // returns the size of the map (dim * resolution)
        std::pair <double, double> getSize();
        bool loadFromFile(std::string file_name);
        void visualize();
    private:
        OccupancyGrid prob_;
        size_t dim_x_;
        size_t dim_y_;
        double resolution_;
        std::vector<size_t> map_min_;
        std::vector<size_t> map_max_;
    };
}