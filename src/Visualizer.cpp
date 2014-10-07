#include <pf/Visualizer.h>

using namespace pf;
using namespace cv;

Visualizer::Visualizer(std::string windowname,
                       MapPtr _map) :
    map_(_map),
    window_name_(windowname)
{
    // Initialize the internal image and start the window thread
    cv::startWindowThread();
    cv::namedWindow(window_name_.c_str(), WINDOW_AUTOSIZE);
    OccupancyGrid grid = std::move(map_->getCroppedMap());
    std::vector<double> data;
    for (int j = static_cast<int>(grid[0].size()-1); j >=0; j--)
        for (int i = 0; i < static_cast<int>(grid.size()); i++)
            data.push_back(grid[i][j]);
    image_.reset(new Mat(grid[0].size(), grid.size(), CV_64FC1, data.data()));
}

// void Visualizer::visualizeArray(OccupancyGrid& grid)
// {
//     imshow(window_name_.c_str(), cv_matrix);
//     // cvWaitKey(0);
// }

void Visualizer::plotRayTrace(const RobotState& robot_state)
{
    // get the map coordinates
    auto d_robot = map_->worldToGrid(robot_state.x(), robot_state.y());
    cv::Point robot(d_robot.first, d_robot.second);
    // get the trace for each angle
    // for now, just go from -90 to 89
    for (int r = -90; r < 89; ++r) {
        double nominal_reading = map_->getNominalReading(robot_state, DEG2RAD(r));
        // convert this to a line and plot
        double new_x = robot_state.x() + nominal_reading*std::cos(
                                normalize_angle(robot_state.theta() + DEG2RAD(r))
                            );
        double new_y = robot_state.y() + nominal_reading*std::sin(
                                normalize_angle(robot_state.theta() + DEG2RAD(r))
                            );
        auto d_new_coords = map_->worldToGrid(new_x, new_y);
        cv::Point range_point(d_new_coords.first, d_new_coords.second);
        line(*image_, robot, range_point, cv::Scalar(0, 255, 255));
    }
    imshow(window_name_, *image_);
}