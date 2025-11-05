#include "WireDetectionSim.h"

#include "util/Timer.h"

namespace rock::log_replay {
    WireDetectionSim::WireDetectionSim() : ClassifySim() {}

    void WireDetectionSim::init() {
        wire_detector_ = std::make_unique<perceptor::detection::wire::WireDetector>();

        if (get_param("replay/sensor_type", 0)) {
            wire_detector_->set_sensor_type(TofHardware::ChaoFeng);
        } else {
            wire_detector_->set_sensor_type(TofHardware::OMS);
        }

        cone_wrapper_.init(nh(), "/cone", "base_link");
        origin_wrapper_.init(nh(), "/origin_image");
        detect_wrapper_.init(nh(), "/detect_image");
        static_map_wrapper_.init(nh(), "/user_map");
        wire_points_wrapper_.init(nh(), "/wire_points", "base_link");

        ClassifySim::init();
        wire_detector_->init();
    }

    void WireDetectionSim::process_cones() {
        if (wire_detector_->status() != perceptor::detection::wire::Status::UpdatedTarget
            && wire_detector_->status() != perceptor::detection::wire::Status::SyncingTof) {
            return;
        }

        sensor_data_t       polygon;
        log_parser::Point3D pt;

        for (const auto& cone : wire_detector_->cones()) {
            polygon.timestamp = wire_detector_->timestamp();

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

    void WireDetectionSim::run() {
        ROS_INFO("begin ==============");
        rock::perceptor::util::TimerGuard timer_all("[run] wire_detection_thread", true);

        auto source_dir = SOURCE_DIR;

        auto last_observation_timestamp = double{-1};

        auto alpha = 0.5f;

        display_delay_ = false;

        auto frames      = 0;
        auto wire_frames = 0;

        std::vector<uint8_t> mask;

        while (auto data = log_parser()->get_data()) {
            auto& raw = data->get();
            marker_wrapper_.process(raw);

#if ENABLE_MASK
            wire_detector_->enable_mask(true);
            ROS_DEBUG("enable_mask_: true");

            if (raw.type == log_data_type_t::Map) {
                static_map_wrapper_.process(raw);
                continue;
            }

            if (raw.type == log_data_type_t ::SlamPose) {
                pose_wrapper_.process(raw);
                rock_pose_2d_stamped_t pose;
                pose.timestamp          = raw.timestamp;
                pose.pose.theta         = raw.position_2d.pos.theta;
                pose.pose.point.x.metre = raw.position_2d.pos.x;
                pose.pose.point.y.metre = raw.position_2d.pos.y;

                algorithm_->update_pose(pose);
                wire_detector_->update_pose(pose);

                if (wire_detector_->status() == perceptor::detection::wire::Status::SyncingTarget) {
                    wire_detector_->sync_target();
                }

                continue;
            }

            if (raw.type == log_data_type_t::Tof && raw.tof.exposure == tof_t::Exposure::HdrFlood) {
                flood_wrapper_.process(raw);

                reference_wrapper_.process(reference_points_, raw.timestamp);

                ++frames;

                auto observation = ObservationConverter::convert_to_tof(raw);
                algorithm_->update_point_cloud(*observation);
                last_observation_timestamp = raw.timestamp;

                if (!algorithm_->run()) {
                    continue;
                }

                classify_wrapper_.process(algorithm_->get_cloud());

                wire_detector_->update_point_cloud(*algorithm_->get_points());
                wire_detector_->wire_detection();

                rock::perceptor::util::Timer::PrintAll();

                wire_points_wrapper_.process(wire_detector_->wire_cloud());

                for (auto pt : wire_detector_->wire_cloud_2d()) {
                    auto point = cv::Point((int)pt.x, (int)pt.y);
                    cv::circle(image_, point, 5, (255, 0, 0), -1);
                }

                auto cloud = wire_detector_->wire_cloud();

                if (cloud->size() > 0) {
                    detect_wrapper_.process(image_, raw.timestamp);
                    ++wire_frames;
                }
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

                    for (auto i = 0; i < image_.rows; ++i) {
                        for (auto j = 0; j < image_.cols; ++j) {
                            auto it    = image_.cols * i + j;
                            auto label = wire_detector_->mask()[it];

                            if (label == 0) {
                                continue;
                            }

                            cv::Vec3b point            = image_.at<cv::Vec3b>(i, j);
                            cv::Vec3b color            = alpha * cv::Vec3b(0, 0, 255) + (1 - alpha) * point;
                            image_.at<cv::Vec3b>(i, j) = color;
                        }
                    }

                    detect_wrapper_.process(image_, raw.timestamp);
                }

                continue;
            }

            if (raw.type == log_data_type_t::WireMask) {
                display_delay_ = true;

                auto mask_encode = raw.mask.mask;
                mask.clear();

                for (auto i = 0; i < mask_encode.size() / 2; ++i) {
                    auto value_idx = 2 * i;
                    auto count_idx = 2 * i + 1;

                    for (auto j = 0; j < mask_encode[count_idx]; ++j) {
                        mask.push_back(static_cast<uint8_t>(mask_encode[value_idx]));
                    }
                }

                for (auto i = 0; i < image_.rows; ++i) {
                    for (auto j = 0; j < image_.cols; ++j) {
                        auto      it    = image_.cols * i + j;
                        auto   label = mask[it];

                        if (label == 0) {
                            continue;
                        }
                            
                        cv::Vec3b point = image_.at<cv::Vec3b>(i, j);
                        cv::Vec3b color            = alpha * cv::Vec3b(0, 0, 255) + (1 - alpha) * point;
                        image_.at<cv::Vec3b>(i, j) = color;
                    }
                }

                wire_detector_->update_mask(mask, raw.timestamp);
                detect_wrapper_.process(image_, raw.timestamp);
            }
#else
            wire_detector_->enable_mask(false);
            ROS_DEBUG("enable_mask_: false");

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
                wire_detector_->update_pose(pose);

                if (wire_detector_->status() == perceptor::detection::wire::Status::SyncingTarget) {
                    wire_detector_->sync_target();
                    process_cones();
                }

                continue;
            }

            if (raw.type == log_data_type_t::Tof && raw.tof.exposure == tof_t::Exposure::HdrFlood) {
                flood_wrapper_.process(raw);

                reference_wrapper_.process(reference_points_, raw.timestamp);

                ++frames;

                auto observation = ObservationConverter::convert_to_tof(raw);
                algorithm_->update_point_cloud(*observation);
                last_observation_timestamp = raw.timestamp;

                if (!algorithm_->run()) {
                    continue;
                }

                classify_wrapper_.process(algorithm_->get_cloud());

                wire_detector_->update_point_cloud(*algorithm_->get_points());
                wire_detector_->wire_detection();

                rock::perceptor::util::Timer::PrintAll();

                wire_points_wrapper_.process(wire_detector_->wire_cloud());

                auto cloud = wire_detector_->wire_cloud();

                if (cloud->size() > 0) {
                    ++wire_frames;
                }
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

            if (raw.type == log_data_type_t::WireRect) {
                display_delay_ = true;
                line_points_.clear();

                auto rects = raw.rect;

                std::vector<rect_points_t> cone_rects;

                for (auto i = 0; i < rects.points.size(); i += 4) {
                    rect_points_t rect;
                    for (auto j = 0; j < 4; ++j) {
                        rect.vertexes.point[j].x.metre = rects.points[i + j].x;
                        rect.vertexes.point[j].y.metre = rects.points[i + j].y;

                        auto p1 = cv::Point2f(rects.points[i + j].x, rects.points[i + j].y);
                        auto p2 = cv::Point2f(rects.points[(i + j + 1) % 4].x, rects.points[(i + j + 1) % 4].y);

                        cv::line(image_, p1, p2, cv::Scalar(0, 255, 0), 2);

                        line_points_.emplace_back(p1);
                        line_points_.emplace_back(p2);
                    }
                    cone_rects.emplace_back(rect);
                }

                wire_detector_->update_cones(cone_rects, raw.timestamp);

                process_cones();

                detect_wrapper_.process(image_, raw.timestamp);
            }

#endif
        }
        std::cout << "cloud frames: " << frames << std::endl;
        std::cout << "wire frames: " << wire_frames << std::endl;
        ROS_INFO("end ==============");
    }
}    // namespace rock::log_replay
