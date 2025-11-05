#include "LogReplay.h"

namespace rock::log_replay {
    LogReplay::LogReplay() : RosInterface() {}

    void LogReplay::init() {
        if (get_param("default/tof_map", true)) {
            map_wrapper_.init(nh(), "/tof_flood");
        }

        pose_wrapper_.init(nh(), "/pose");
        icon_wrapper_.init(nh(), "/icon");
        image_wrapper_.init(nh(), "/image");
        spot_wrapper_.init(nh(), "/tof_spot");
        wire_wrapper_.init(nh(), "/tof_wire");
        flood_wrapper_.init(nh(), "/tof_flood");
        top_tof_wrapper_.init(nh(), "/top_tof_flood");
        cliff_wrapper_.init(nh(), "/cliff");
        marker_wrapper_.init(nh(), "/time_display");
        static_map_wrapper_.init(nh(), "/user_map");
        polygon_wrapper_.init(nh(), "/obs_polygons");
        camera2d_wrapper_.init(nh(), "/camera2d");
        obs_map_wrapper_.init(nh(), "/obs_map");
        detect_wrapper_.init(nh(), "/detect_image");
        shadow_laser_wrapper_.init(nh(), "/shadow_laser");

        std::vector<log_path_t> file_to_parse;
        get_file_list(file_to_parse);

        log_parser()->set_log_time(get_param("replay/log_start", 0));

        log_parser()->init(file_to_parse);

        log_parser()->begin();
    }

    void LogReplay::run() {
        ROS_INFO_STREAM("Log replay node start");
        while (auto data = log_parser()->get_data()) {
            auto& raw = data->get();
            marker_wrapper_.process(raw);

            switch (raw.type) {
                case log_data_type_t::SlamPose: {
                    map_wrapper_.process(raw);
                    pose_wrapper_.process(raw);
                    break;
                }
                case log_data_type_t::Tof: {
                    if (raw.tof.exposure == tof_t::Exposure::HdrFlood) {
                        flood_wrapper_.process(raw);
                    } else if (raw.tof.exposure == tof_t::Exposure::HdrSpot
                               && raw.sensor == static_cast<int>(tof_t::Sensor::Front)) {
                        spot_wrapper_.process(raw);
                    } else if (raw.tof.exposure == tof_t::Exposure::Wire) {
                        wire_wrapper_.process(raw);
                    }
                    break;
                }
                case log_data_type_t::Tof3DMap: {
                    top_tof_wrapper_.process(raw);
                    break;
                }
                case log_data_type_t::Cliff: {
                    cliff_wrapper_.process(raw);
                    break;
                }
                case log_data_type_t::Camera_Image: {
                    image_ = cv::imread(raw.camera.name, cv::IMREAD_COLOR);
                    image_wrapper_.process(raw);
                    break;
                }
                case log_data_type_t::Icon: {
                    icon_wrapper_.process(raw);
                    break;
                }
                case log_data_type_t::Map: {
                    static_map_wrapper_.process(raw);
                    break;
                }
                case log_data_type_t::Nav: {
                    if (raw.sensor == static_cast<int>(nav_t::Type::Camera2d)) {
                        camera2d_wrapper_.process(raw);
                    } else if (raw.sensor == static_cast<int>(nav_t::Type::Obs_map)) {
                        obs_map_wrapper_.process(raw);
                    } else if (raw.sensor == static_cast<int>(nav_t::Type::Polygons)) {
                        polygon_wrapper_.process(raw);
                    } else if (raw.sensor == static_cast<int>(nav_t::Type::Shadow_laser)) {
                        shadow_laser_wrapper_.process(raw);
                    }
                    break;
                }

                case log_data_type_t::WireRect: {
                    auto& rects = raw.rect;

                    for (auto i = 0; i < rects.points.size(); i += 4) {
                        rect_points_t rect;
                        for (auto j = 0; j < 4; ++j) {
                            auto p1 = cv::Point2f(rects.points[i + j].x * 800 / 640, rects.points[i + j].y * 600 / 480);
                            auto p2 = cv::Point2f(rects.points[(i + j + 1) % 4].x * 800 / 640,
                                                  rects.points[(i + j + 1) % 4].y * 600 / 480);

                            cv::line(image_, p1, p2, cv::Scalar(0, 255, 0), 2);
                        }
                    }

                    detect_wrapper_.process(image_, raw.timestamp);
                    break;
                }

                default:
                    break;
            }
        }
    }
}    // namespace rock::log_replay
