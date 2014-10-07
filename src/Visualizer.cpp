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
    OccupancyGrid grid = map_->getMap();
    // std::vector<double> data;
    // for (int j = static_cast<int>(grid[0].size()-1); j >=0; j--) {
    //     for (int i = 0; i < static_cast<int>(grid.size()); i++) {
    //         data.push_back(grid[i][j]);
    //     }
    // }
    // Mat tmp_mat(grid[0].size(), grid.size(), CV_64FC1, data.data());
    dim_x_ = grid.size();
    dim_y_ = grid[0].size();
    map_img_ = cv::Mat(dim_y_, dim_x_, CV_8UC3);
    for (int i = 0; i < static_cast<int>(dim_x_); i++) {
        for (int j = 0; j < static_cast<int>(dim_y_); j++) {
            if (grid[i][j] == -1) {
                map_img_.at<Vec3b>(dim_y_ - j - 1, i)[0] = 0;
                map_img_.at<Vec3b>(dim_y_ - j - 1, i)[1] = 0;
                map_img_.at<Vec3b>(dim_y_ - j - 1, i)[2] = 0;
            } else {
                map_img_.at<Vec3b>(dim_y_ - j - 1, i)[0] = 255*grid[i][j];
                map_img_.at<Vec3b>(dim_y_ - j - 1, i)[1] = 255*grid[i][j];
                map_img_.at<Vec3b>(dim_y_ - j - 1, i)[2] = 255*grid[i][j];
            }
        }
    }
}


void Visualizer::showMap()
{
    Mat current_image = map_img_.clone();
    imshow(window_name_.c_str(), map_img_);
    // waitKey(0);
}

void Visualizer::plotRayTrace(const RobotState& robot_state)
{
    Mat current_image = map_img_.clone();
    // get the map coordinates
    auto d_robot = map_->worldToGrid(robot_state.x(), robot_state.y());
    cv::Point robot(d_robot.first, dim_y_ - d_robot.second);
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
        cv::Point range_point(d_new_coords.first, dim_y_ - d_new_coords.second);
        line(current_image, robot, range_point, cv::Scalar(0, 255, 255));
    }
    imshow(window_name_.c_str(), current_image);
}