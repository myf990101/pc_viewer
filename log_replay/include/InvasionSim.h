#pragma once

#include <Eigen/Core>

#include "detection/InvasionDetector.h"
#include "detection/TopClassify.hpp"
#include "interface/observation_converter.hpp"
#include "interface/ros_interface.hpp"

namespace rock::log_replay {
    using point_3d_t = rock::perceptor::common::Point3D<float>;
    using vec3f_t    = Eigen::Vector3f;
    using points_t   = std::vector<vec3f_t, Eigen::aligned_allocator<vec3f_t>>;
    class InvasionSim : public RosInterface {
    public:
        InvasionSim();

        ~InvasionSim() = default;

        void init() override;

        void run() override;

        void process_cones(uint32_t timestamp, points_t cube, rock::log_replay::ros_wrapper::Polygon& cone_wrapper);

    private:
        std::unique_ptr<rock::perceptor::detection::invasion::InvasionDetector> invasion_detector_;
        std::unique_ptr<rock::perceptor::detection::topclassify::TopClassify>   top_tof_classify_;

        ros_wrapper::Polygon    cone_wrapper_;
        ros_wrapper::Polygon    cone_wrapper2_;
        ros_wrapper::Polygon    cone_wrapper3_;
        ros_wrapper::Polygon    cone_wrapper4_;
        ros_wrapper::Pose       pose_wrapper_;
        ros_wrapper::Image      origin_wrapper_;
        ros_wrapper::Marker     marker_wrapper_;
        ros_wrapper::PointCloud flood_wrapper_;
        ros_wrapper::PointCloud classify_wrapper_;
        ros_wrapper::PointCloud invasion_wrapper_;
    };
}    // namespace rock::log_replay
