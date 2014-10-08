#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pf/Constants.h>
#include <pf/Map.h>

namespace pf {
    class Visualizer {
    public:
        Visualizer(std::string windowname, MapPtr map);
        void setWindowName(std::string name) { window_name_ = name; }
        void plotRayTrace(const RobotState& robot_state, std::vector<double> bearings);
        void visualizePoses(const std::vector<RobotState>& robot_states);
        void visualizeScan(const RobotState& robot_state, 
                            const std::vector<double>& scan_data);
        void showMap();
        void visualizeOnlyScan(const std::vector<double>& scan_data);
        void testMap();
    private:
        void visualizeRobotPose(cv::Mat& current_image, const RobotState& state);
        MapPtr map_;
        std::string window_name_;
        std::string scan_window_name_;
        cv::Mat map_img_;
        cv::Mat current_image_;
        size_t dim_x_;
        size_t dim_y_;
    };
    typedef std::shared_ptr<Visualizer> VizPtr;
}