#pragma once

#include "ClassifySim.h"
#include "detection/PostFusion.h"

namespace rock::log_replay {
    class PostFusionSim : protected ClassifySim {
    public:
        PostFusionSim();

        ~PostFusionSim() = default;

    public:
        void init() override;

        void run();

    private:
        void process_cones();

    private:
        std::unique_ptr<perceptor::detection::post::PostFusion> post_fusion_;

        cv::Mat                 image_;
        ros_wrapper::Polygon    cone_wrapper_;
        ros_wrapper::Image      origin_wrapper_;
        ros_wrapper::Image      detect_wrapper_;
        ros_wrapper::StaticMap  static_map_wrapper_;
        ros_wrapper::PointCloud fusion_points_wrapper_;

        bool                     display_delay_;
        std::vector<cv::Point2f> line_points_;
        std::vector<uint8_t>     mask_;
    };
}    // namespace rock::log_replay
