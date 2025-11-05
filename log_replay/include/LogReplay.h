#pragma once

#include "interface/ros_interface.hpp"

namespace rock::log_replay {
    class LogReplay : public RosInterface {
    public:
        LogReplay();

        ~LogReplay() = default;

        void init() override;

        void run() override;

    private:
        cv::Mat image_;

        ros_wrapper::Map        map_wrapper_;
        ros_wrapper::Pose       pose_wrapper_;
        ros_wrapper::Icon       icon_wrapper_;
        ros_wrapper::Image      image_wrapper_;
        ros_wrapper::Image      detect_wrapper_;
        ros_wrapper::Cliff      cliff_wrapper_;
        ros_wrapper::Marker     marker_wrapper_;
        ros_wrapper::Polygons   polygon_wrapper_;
        ros_wrapper::NavPoints  camera2d_wrapper_;
        ros_wrapper::NavPoints  obs_map_wrapper_;
        ros_wrapper::NavPoints  shadow_laser_wrapper_;
        ros_wrapper::StaticMap  static_map_wrapper_;
        ros_wrapper::PointCloud spot_wrapper_;
        ros_wrapper::PointCloud wire_wrapper_;
        ros_wrapper::PointCloud flood_wrapper_;
        ros_wrapper::PointCloud top_tof_wrapper_;
    };
}    // namespace rock::log_replay
