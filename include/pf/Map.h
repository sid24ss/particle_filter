#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include <pf/Constants.h>
#include <pf/RobotState.h>

namespace pf {
    /**
     * @brief contains the map
     */
    class Map {
    public:
        Map();
        Map(std::string file_name);
        // returns the dimensions of the cells underneath.
        std::pair <size_t, size_t> getDims() const;
        // returns the size of the map (dim * resolution)
        std::pair <double, double> getSize() const;
        // loads the map from file
        bool loadFromFile(std::string file_name);
        // get the underlying occupancy grids
        OccupancyGrid getCroppedMap() const;
        OccupancyGrid getMap() const { return prob_; };
        // convert the (x,y) of the world to the (x,y) of the grid.
        std::pair<size_t, size_t> worldToGrid(double x, double y) const;
        // convert the (x,y) of the grid to the (x,y) of the world.
        std::pair<double, double> gridToWorld(size_t x, size_t y) const;
        // is within range
        bool withinRange(double x, double y);
        // position within range, not in collision, not in unknown space
        bool isFree(double x, double y);
        // ray tracing functions
        double getNominalReading(const RobotState& robot_state, double bearing);
    private:
        // The underlying structure that stores the grid
        OccupancyGrid prob_;
        // dim_ is the integer size of the map. eg., 759 x 405
        size_t dim_x_;
        size_t dim_y_;
        double resolution_;
        std::vector<size_t> map_min_;
        std::vector<size_t> map_max_;
    };
    typedef std::shared_ptr<Map> MapPtr;
}
