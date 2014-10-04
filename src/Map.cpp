#include <cstdio>

#include <boost/progress.hpp>

#include <pf/Map.h>
#include <pf/Visualizer.h>

using namespace pf;

Map::Map() :
    map_min_(2, 0),
    map_max_(2, 0)
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
    boost::progress_display show_progress(dim_x_*dim_y_);
    map_min_[RobotDOF::X] = dim_x_;
    map_min_[RobotDOF::Y] = dim_y_;
    OccupancyGrid raw_map;
    raw_map.resize(dim_x_);
    for (size_t x = 0; x < dim_x_; ++x) {
        raw_map[x].resize(dim_y_);
        for (size_t y = 0; y < dim_y_; ++y) {
            if (fscanf(fin, "%lf ", &raw_map[x][y]) <= 0)
                return false;
            if (raw_map[x][y] >= 0.0f) {
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
    // crop the map
    prob_.resize(map_max_[RobotDOF::X] - map_min_[RobotDOF::X] + 1);
    for (size_t x = map_min_[RobotDOF::X]; x <= map_max_[RobotDOF::X]; ++x){
        prob_[x-map_min_[RobotDOF::X]].resize(map_max_[RobotDOF::Y] - map_min_[RobotDOF::Y] + 1);
        for (size_t y = map_min_[RobotDOF::Y]; y <= map_max_[RobotDOF::Y]; ++y){
            prob_[x-map_min_[RobotDOF::X]][y - map_min_[RobotDOF::Y]] = raw_map[x][y];
        }
    }
    return true;
}

std::pair <size_t, size_t> Map::getDims()
{
    return std::make_pair(dim_x_, dim_y_);
}

std::pair <double, double> Map::getSize()
{
    return std::make_pair(dim_x_*resolution_, dim_y_*resolution_);
}

void Map::visualize()
{
    printf("visualizing map\n");
    Visualizer::visualizeArray(prob_);
}