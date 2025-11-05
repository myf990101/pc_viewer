#pragma once

#include <Eigen/Core>

#include "classify/Classify.h"
#include "common/calibration.h"
#include "common/SensorExtrinsic.h"
#include "interface/observation_converter.hpp"
#include "interface/ros_interface.hpp"

namespace rock::log_replay {
    constexpr auto TofPosition = float{0.16};
    // using tof_classify_type   = perceptor::tof::Classify2;
    using tof_classify_type   = perceptor::tof::Classify;

    class ClassifySim : public RosInterface {
    public:
        using sensor_extrinsic_t = rock::perceptor::common::SensorExtrinsic;

    public:
        ClassifySim();

        ~ClassifySim() = default;

        void init() override;

        void run() override;

    protected:
        void prepare_reference_points();

        auto process_tof_slc(log_parser::Nav::TofSLC& data) -> sensor_extrinsic_t;

        auto compute_delta_rotation(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) -> Eigen::Matrix3d;

    protected:
        std::unique_ptr<tof_classify_type> algorithm_;

        std::vector<cv::Point3f> reference_points_;

        ros_wrapper::Pose       pose_wrapper_;
        ros_wrapper::Image      image_wrapper_;
        ros_wrapper::Marker     marker_wrapper_;
        ros_wrapper::PointCloud flood_wrapper_;
        ros_wrapper::PointCloud classify_wrapper_;
        ros_wrapper::PointCloud reference_wrapper_;
    };
}    // namespace rock::log_replay
