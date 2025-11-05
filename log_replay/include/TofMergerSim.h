#pragma once

#include "common/Types.h"
#include "interface/observation_converter.hpp"
#include "interface/ros_interface.hpp"
#include "tof_merger/TofMerger.h"

namespace rock::log_replay {
    class TofMergerSim : public RosInterface {
    public:
        using tof_multi_line_param_type = perceptor::tof::TofMerger::tof_multi_line_param_ptr_t;

    public:
        TofMergerSim();

        ~TofMergerSim() = default;

        void init() override;

        void run() override;

    private:
        std::unique_ptr<perceptor::tof::TofMerger> algorithm_;

        ros_wrapper::Pose       pose_wrapper_;
        ros_wrapper::Marker     marker_wrapper_;
        ros_wrapper::PointCloud front_wrapper_;
        ros_wrapper::PointCloud left_wrapper_;
        ros_wrapper::PointCloud merge_wrapper_;
        ros_wrapper::PointCloud laser4_wrapper_;

        ros_wrapper::PointCloud valid_wrapper_;
        ros_wrapper::PointCloud filter_wrapper_;
        ros_wrapper::PointCloud points_classify_wrapper_;

        perceptor::tof::TofMerger::tof_multi_line_param_ptr_t multi_line_param_ptr;
    };
}    // namespace rock::log_replay
