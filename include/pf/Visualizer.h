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
        void plotRayTrace(const RobotState& robot_state);
        void showMap();
    private:
        MapPtr map_;
        std::string window_name_;
        cv::Mat map_img_;
    };
    typedef std::shared_ptr<Visualizer> VizPtr;
}