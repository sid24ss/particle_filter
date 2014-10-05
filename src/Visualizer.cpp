#include <pf/Visualizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace pf;
using namespace cv;

std::string Visualizer::window_name_ = std::string("pf_viz");

void Visualizer::visualizeArray(OccupancyGrid& grid)
{
    std::vector<double> data;
    for (int j = static_cast<int>(grid[0].size()-1); j >=0; j--)
        for (int i = 0; i < static_cast<int>(grid.size()); i++)
            data.push_back(grid[i][j]);
    Mat cv_matrix(grid[0].size(), grid.size(), CV_64FC1, data.data());
    namedWindow(window_name_.c_str(), WINDOW_AUTOSIZE);
    imshow(window_name_.c_str(), cv_matrix);
    cvWaitKey(0);
}
