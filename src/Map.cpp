#include <cstdio>
#include <cassert>
#include <cmath>

#include <boost/progress.hpp>

#include <pf/Map.h>
#include <pf/Utilities.h>

using namespace pf;

Map::Map() :
    map_min_(2,0),
    map_max_(2,0)
{}

bool Map::loadFromFile(std::string file_name)
{
    FILE* fin = fopen(file_name.c_str(), "r");
    if (!fin) {
        printf("file (%s) does not exist\n", file_name.c_str());
        return false;
    }
    double size_x;
    double size_y;
    double autoshift_x;
    double autoshift_y;

    if(fscanf(fin, "robot_specifications->global_mapsize_x                 %lf\n",
            &size_x) <= 0)
        return false;
    if(fscanf(fin, "robot_specifications->global_mapsize_y                 %lf\n",
            &size_y) <= 0)
        return false;
    if(fscanf(fin, "robot_specifications->resolution                       %lf\n",
            &resolution_) <= 0)
        return false;
    if(fscanf(fin, "robot_specifications->autoshifted_x                    %lf\n",
            &autoshift_x) <= 0)
        return false;
    if(fscanf(fin, "robot_specifications->autoshifted_y                    %lf\n",
            &autoshift_y) <= 0)
        return false;
    // read an empty line
    fscanf(fin, "\n");
    if(fscanf(fin, "global_map[0]: %lu %lu\n", &dim_x_, &dim_y_) <= 0)
        return false;
    printf("dim_x_\t: %lu\n", dim_x_);
    printf("dim_y_\t: %lu\n", dim_y_);
    printf("resolution_\t: %f\n", resolution_);

    // initialize to extremes
    map_min_[RobotDOF::X] = dim_x_;
    map_min_[RobotDOF::Y] = dim_y_;

    prob_.resize(dim_x_);
    boost::progress_display show_progress(dim_x_*dim_y_);
    for (size_t x = 0; x < dim_x_; ++x) {
        prob_[x].resize(dim_y_);
        for (size_t y = 0; y < dim_y_; ++y) {
            if (fscanf(fin, "%lf ", &prob_[x][y]) <= 0)
                return false;
            if (prob_[x][y] >= 0.0f) {
                if (map_min_[RobotDOF::X] > x)
                    map_min_[RobotDOF::X] = x;
                if (map_max_[RobotDOF::X] < x)
                    map_max_[RobotDOF::X] = x;
                if (map_min_[RobotDOF::Y] > y)
                    map_min_[RobotDOF::Y] = y;
                if (map_max_[RobotDOF::Y] < y)
                    map_max_[RobotDOF::Y] = y;
            }
            ++show_progress;
        }
    }
    return true;
}

std::pair <size_t, size_t> Map::getDims() const
{
    return std::make_pair(dim_x_, dim_y_);
}

std::pair <double, double> Map::getSize() const
{
    return std::make_pair(dim_x_*resolution_, dim_y_*resolution_);
}

/**
 * @brief converts the continuous coordinates to the grid coordinates
 * 
 * @param x x in the world
 * @param y y in the world
 * 
 * @return a pair (x,y) in the grid
 */
std::pair<size_t, size_t> Map::worldToGrid(double x, double y) const
{
    size_t x_d = static_cast<size_t>(x/resolution_ + 0.5);
    size_t y_d = static_cast<size_t>(y/resolution_ + 0.5);
    // if it exceeds the dimensions, snap to the last valid state.
    if (x_d == dim_x_)
        x_d--;
    if (y_d == dim_y_)
        y_d--;
    // if (x_d == -1)
    //     x_d++;
    // if (y_d == -1)
    //     y_d++;
    assert(x_d >= 0 && x_d < dim_x_ && y_d >=0 && y_d < dim_y_);
    return std::make_pair(x_d, y_d);
}

/**
 * @brief converts the discrete map coordinates to the continuous world coords
 * @details Can only project to the closest in the resolution. Typically, you
 * don't want to be going this way.
 * 
 * @param x_d the discrete x coordinate
 * @param y_d the discrete y coordinate
 * 
 * @return a pair (x, y) in the world
 */
std::pair<double, double> Map::gridToWorld(size_t x_d, size_t y_d) const
{
    double x = static_cast<double>(x_d) * resolution_;
    double y = static_cast<double>(y_d) * resolution_;
    return std::make_pair(x, y);
}

bool Map::withinRange(double x, double y)
{
    return (x >= 0 && x < dim_x_*resolution_
        &&  y >= 0 && y < dim_y_*resolution_);
}

bool Map::isFree(double x, double y)
{
    std::pair<size_t, size_t> coords = worldToGrid(x, y);
    if (withinRange(x, y) &
        prob_[coords.first][coords.second] <= MapParams::FREE_THRESHOLD & 
        prob_[coords.first][coords.second] >= 0)
        return true;
    return false;
}

OccupancyGrid Map::getCroppedMap() const
{
    OccupancyGrid data;
    // crop the map
    data.resize(map_max_[RobotDOF::X] - map_min_[RobotDOF::X] + 1);
    for (size_t x = map_min_[RobotDOF::X]; x <= map_max_[RobotDOF::X]; ++x){
        data[x-map_min_[RobotDOF::X]].resize(map_max_[RobotDOF::Y] - map_min_[RobotDOF::Y] + 1);
        for (size_t y = map_min_[RobotDOF::Y]; y <= map_max_[RobotDOF::Y]; ++y){
            data[x-map_min_[RobotDOF::X]][y - map_min_[RobotDOF::Y]] = prob_[x][y];
        }
    }
    return data;
    // Visualizer::visualizeArray(data);
}

/**
 * @brief does ray tracing
 * @details Give a RobotState (that is, x, y, theta) and a bearing (the angle 
 * of the lidar reading), we want to compute the nominal range where this ray
 * meets the map. Note that the RobotState is in the world coordinates. That is,
 * the (x,y) are real values, not cells.
 * 
 * @param robot_state the current pose
 * @param bearing The laser bearing IN RADIANS
 * 
 * @return The nominal range where the ray hits the map
 */
double Map::getNominalReading(const RobotState& robot_state, double bearing)
{
    // first, compute the actual direction in which we need to march.
    double angle = normalize_angle(robot_state.theta() + bearing);
    // printf("marching along global angle : %f\n", RAD2DEG(angle));
    // from (x, y), we need to go along this angle until we hit a wall
    // The resolution of the map is resolution_
    // therefore, we need the values at the points
    // x + 5*cos(angle), y + 5*sin(angle)
    // x + 15*cos(angle), y + 15*sin(angle), ...
    double current_distance = 0.0;
    double x0 = robot_state.x();
    double y0 = robot_state.y();
    // current coordinates in the world
    double x = x0;
    double y = y0;
    // current equivalent map coordinates
    std::pair<size_t, size_t> map_current_coords = worldToGrid(x, y);
    // printf("(Initial prob : %f)\n", prob_[map_current_coords.first][map_current_coords.second]);
    do {
        current_distance += resolution_;
        x = x0 + current_distance*std::cos(angle);
        y = y0 + current_distance*std::sin(angle);
        // printf("current x : %f, y : %f\n", x, y);
        map_current_coords = worldToGrid(x, y);
    } while (
        // prob_[map_current_coords.first][map_current_coords.second] <
        //                                             MapParams::WALL_THRESHOLD
    prob_[map_current_coords.first][map_current_coords.second] >= 0.1
    && withinRange(x, y));
    return current_distance;
}
