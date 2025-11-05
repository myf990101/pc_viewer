#include "PostFusionSim.h"

#include "util/Timer.h"

namespace rock::log_replay {
    PostFusionSim::PostFusionSim() : ClassifySim() {}

    void PostFusionSim::init() {
        post_fusion_ = std::make_unique<perceptor::detection::post::PostFusion>();

        cone_wrapper_.init(nh(), "/cone", "base_link");
        origin_wrapper_.init(nh(), "/origin_image");
        detect_wrapper_.init(nh(), "/detect_image");
        static_map_wrapper_.init(nh(), "/user_map");
        fusion_points_wrapper_.init(nh(), "/fusion_points", "base_link");

        ClassifySim::init();
        post_fusion_->init();
    }

    void PostFusionSim::process_cones() {
        if (post_fusion_->status() != perceptor::detection::post::Status::UpdatedTarget
            && post_fusion_->status() != perceptor::detection::post::Status::SyncingTof) {
            return;
        }

        sensor_data_t       polygon;
        log_parser::Point3D pt;

        for (const auto& cone : post_fusion_->cones()) {
            polygon.timestamp = post_fusion_->timestamp();
            polygon.nav.points.clear();

            for (const auto& p : cone.front_points) {
                pt.x = p.x();
                pt.y = p.y();
                pt.z = p.z();
                polygon.nav.points.emplace_back(pt);
            }

            polygon.nav.points.emplace_back(polygon.nav.points.front());

            for (const auto& p : cone.back_points) {
                pt.x = p.x();
                pt.y = p.y();
                pt.z = p.z();
                polygon.nav.points.emplace_back(pt);
            }

            //connect polygon together
            polygon.nav.points.emplace_back(polygon.nav.points[5]);
            polygon.nav.points.emplace_back(polygon.nav.points[6]);
            polygon.nav.points.emplace_back(polygon.nav.points[1]);
            polygon.nav.points.emplace_back(polygon.nav.points[2]);
            polygon.nav.points.emplace_back(polygon.nav.points[7]);
            polygon.nav.points.emplace_back(polygon.nav.points[8]);
            polygon.nav.points.emplace_back(polygon.nav.points[3]);
        }

        cone_wrapper_.process(polygon);
    }

    void PostFusionSim::run() {
        ROS_INFO("begin ==============");
        rock::perceptor::util::TimerGuard timer_all("[run] post_fusion_thread", true);

        auto source_dir = SOURCE_DIR;

        auto last_observation_timestamp = double{-1};

        display_delay_ = false;

        auto frames     = 0;
        auto obj_frames = 0;

        while (auto data = log_parser()->get_data()) {
            auto& raw = data->get();
            marker_wrapper_.process(raw);

            if (raw.type == log_data_type_t::Map) {
                static_map_wrapper_.process(raw);
                continue;
            }

            if (raw.type == log_data_type_t ::SlamPose) {
                pose_wrapper_.process(raw);
                pose_2d_stamped_t pose;
                pose.timestamp          = raw.timestamp;
                pose.pose.theta         = raw.position_2d.pos.theta;
                pose.pose.point.x.metre = raw.position_2d.pos.x;
                pose.pose.point.y.metre = raw.position_2d.pos.y;

                algorithm_->update_pose(pose);
                post_fusion_->update_pose(pose);

                if (post_fusion_->status() == perceptor::detection::post::Status::SyncingTarget) {
                    post_fusion_->sync_target();
                    process_cones();
                }

                continue;
            }

            if (raw.type == log_data_type_t::Tof && raw.tof.exposure == tof_t::Exposure::HdrFlood) {
                flood_wrapper_.process(raw);

                reference_wrapper_.process(reference_points_, raw.timestamp);

                ++frames;

                auto observation = ObservationConverter::convert_to_tof(raw);
                post_fusion_->update_point_cloud(*observation);
                last_observation_timestamp = raw.timestamp;

                post_fusion_->post_fusion();

                fusion_points_wrapper_.process(post_fusion_->fusion_cloud());

                auto cloud = post_fusion_->fusion_cloud();

                if (cloud->size() > 0) {
                    ++obj_frames;
                }

                rock::perceptor::util::Timer::PrintAll();
                continue;
            }

            if (raw.type == log_data_type_t::Camera_Image) {
                origin_wrapper_.process(raw);
                cv::Mat image_ori = cv::imread(raw.camera.name, cv::IMREAD_COLOR);

                if (image_ori.rows <= 0) {
                    continue;
                }

                cv::resize(image_ori, image_, cv::Size(640, 480));

                if (display_delay_) {
                    display_delay_ = false;

                    cv::Mat image_copy;
                    image_.copyTo(image_copy);

                    for (auto i = 0; i < line_points_.size(); i += 8) {
                        for (auto j = 0; j < 8; j += 2) {
                            const auto& p1 = line_points_[i + j];
                            const auto& p2 = line_points_[i + j + 1];

                            cv::line(image_copy, p1, p2, cv::Scalar(0, 255, 0), 2);
                        }
                    }

                    detect_wrapper_.process(image_copy, raw.timestamp);
                }

                continue;
            }

            if (raw.type == log_data_type_t::ObjRect) {
                display_delay_ = true;
                line_points_.clear();

                auto rects = raw.obj;

                std::vector<obj_point_t> cone_rects;

                for (auto i = 0; i < rects.obj.size(); i += 5) {
                    obj_point_t rect;
                    if (rects.obj.size() % 5 > 0) {
                        continue;
                    }
                    auto type = rects.obj[i + 0];

                    rect.type               = (int32_t)type;
                    rect.rect.point.x.metre = rects.obj[i + 1] * 640 / 800;
                    rect.rect.point.y.metre = rects.obj[i + 2] * 480 / 640;
                    rect.rect.size.width    = rects.obj[i + 3] * 640 / 800;
                    rect.rect.size.height   = rects.obj[i + 4] * 480 / 640;

                    auto p1 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre);
                    auto p2 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width, rect.rect.point.y.metre);

                    cv::line(image_, p1, p2, cv::Scalar(0, 255, 0), 2);

                    auto p3 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width, rect.rect.point.y.metre);
                    auto p4 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width,
                                          rect.rect.point.y.metre + rect.rect.size.height);

                    cv::line(image_, p3, p4, cv::Scalar(0, 255, 0), 2);

                    auto p5 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width,
                                          rect.rect.point.y.metre + rect.rect.size.height);
                    auto p6 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre + rect.rect.size.height);

                    cv::line(image_, p5, p6, cv::Scalar(0, 255, 0), 2);

                    auto p7 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre + rect.rect.size.height);
                    auto p8 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre);

                    cv::line(image_, p7, p8, cv::Scalar(0, 255, 0), 2);

                    line_points_.emplace_back(p1);
                    line_points_.emplace_back(p2);
                    line_points_.emplace_back(p3);
                    line_points_.emplace_back(p4);
                    line_points_.emplace_back(p5);
                    line_points_.emplace_back(p6);
                    line_points_.emplace_back(p7);
                    line_points_.emplace_back(p8);

                    cone_rects.emplace_back(rect);
                }
                
                post_fusion_->update_cones(cone_rects, raw.timestamp);

                process_cones();

                detect_wrapper_.process(image_, raw.timestamp);

                continue;
            }
        }
        std::cout << "cloud frames: " << frames << std::endl;
        ROS_INFO("end ==============");
    }
}    // namespace rock::log_replay
