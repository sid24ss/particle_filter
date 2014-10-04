#pragma once
#include <iostream>
#include <vector>
#include <string>

namespace pf {
    /**
     * @brief contains the map
     */
    class Map {
    public:
        Map() {}
        // returns the dimensions of the cells underneath.
        std::pair <size_t, size_t> getDims();
        // returns the size of the map (dim * resolution)
        std::pair <double, double> getSize();
        bool loadFromFile(std::string file_name);
    private:
        std::vector <std::vector <double> > prob_;
        size_t dim_x_;
        size_t dim_y_;
        double resolution_;
    };
}