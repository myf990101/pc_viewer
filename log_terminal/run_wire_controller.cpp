//
// Created by gaoyuan on 2023/9/11.
//
#include <yaml-cpp/yaml.h>
#include "detection/WireDetector.h"
#include "interface/log_file_definition.hpp"
#include "interface/observation_converter.hpp"
#include "log_parser.hpp"
#include "perceptor.h"
#include "mapping3d/TofMapper.h"
#include "mapprocess/MapWall.hpp"
#include "perceptor_msg_types.h"
#include "util/Chrono.h"
#include "util/Log.h"
#include "util/Timer.h"
#include <cstdint>

extern std::string test_config;
static const auto  WaitForFutureTime = int32_t{50};    //ms

class TestTimestamp {
public:
    static double now() {
        auto duration = std::chrono::steady_clock::now() - instance().actual_start_;
        auto diff
            = (double)duration.count() * decltype(duration)::period::num / decltype(duration)::period::den;
        return instance().test_start_ + diff;
    }

    static void reset(double timestamp) {
        instance().test_start_   = timestamp;
        instance().actual_start_ = std::chrono::steady_clock::now();
    }

private:
    static TestTimestamp& instance() {
        static auto timestamp = TestTimestamp{};
        return timestamp;
    }

    TestTimestamp() = default;

    double                                   test_start_;
    std::chrono::time_point<std::chrono::steady_clock> actual_start_;
};

template<typename Init>
auto make_message(RockPerceptorMessageType type, const uint32_t& size, Init init) -> msg_t const* {
    auto message = (msg_t*)rock_perceptor_memory_allocate(sizeof(msg_t) + size);

    message->type = type;
    message->size = static_cast<uint32_t>(size);
    init(message);

    return message;
}

void callback(msg_t const* message) {
    using namespace rock::perceptor;
    switch (message->type) {
        case RockPerceptorMessageType::Wire: {
            auto observation = *(point_cloud_data_t**)message->data;
            //                auto msg         = rr_msg_tof_points_data_t {};
            //                msg.time_stamp   = observation->timestamp * 1000;
            //                msg.tof_id       = observation->cloud.sensor;
            //                msg.type         = observation->cloud.type;
            //                msg.point_num    = observation->cloud.size;
            //                msg.frame_type   = Wire;
            //                for (int i = 0; i < msg.point_num; ++i) {
            //                    msg.points[i].x          = observation->cloud.points_4d[i].x.metre;
            //                    msg.points[i].y          = observation->cloud.points_4d[i].y.metre;
            //                    msg.points[i].z          = observation->cloud.points_4d[i].z.metre;
            //                    msg.points[i].confidence = observation->cloud.points_4d[i].i.metre;
            //                }
            //                auto& param = *(Parameter*) message->param;
            //                rr_broadcast_msg(param.control, eRRMsgType_TofPercPoints, &msg, sizeof(msg));
            log::INFO_STREAM("receive message... timestamp = " << observation->timestamp
                                                               << ", type = " << observation->type
                                                               << ", sensor = " << observation->sensor
                                                               << ", point num = " << observation->size);

            for (int i = 0; i < observation->size; ++i) {
                const auto& pt = observation->points_classify[i];
                log::INFO_STREAM("cloud " << pt.x.metre << " " << pt.y.metre << " " << pt.z.metre << " " << pt.i.grid);
            }

            break;
        }

        default:
            break;
    }
}

int main(int argc, char** argv) {
    using namespace rock;
    using namespace rock::log_replay;
    using namespace rock::perceptor;
    using namespace rock::perceptor::common;
    using namespace rock::perceptor::detection::wire;

    auto conf          = YAML::LoadFile(WORK_SPACE_LOCATION"/config/wire_detection.yaml");
    auto log_directory = conf["replay"]["log_directory"].as<std::string>();

    test_config = std::string(WORK_SPACE_LOCATION"/config/wire_detection.yaml");

    auto count      = 0u;
    auto paused     = false;
    auto start_time = 88;
    auto alpha      = 0.5f;

    auto log_parser = rock::log_parser::LogParser(
        {{.type = PointCloudDownSamplePath.type, .path = log_directory + PointCloudDownSamplePath.path},
         {.type = CommandPath.type, .path = log_directory + CommandPath.path},
         {.type = SensorLogPath.type, .path = log_directory + SensorLogPath.path},
         {.type = ImagePath.type, .path = log_directory + ImagePath.path},
         {.type = WireRectPath.type, .path = log_directory + WireRectPath.path},
         {.type = WireMaskPath.type, .path = log_directory + WireMaskPath.path}});

    log_parser.set_log_time(start_time);

    log_parser.begin();

    auto time_reset = false;

    cv::Mat rgb_img;
    auto    last_camera_timestamp = timestamp_t{};
    auto*   buffer                = new uint8_t[1];

    std::vector<cv::Point2f> line_points;
    std::vector<uint8_t>     mask;
    // rock_perceptor_enable_3dmapping(false);
    rock_perceptor_controller_init(callback);
    auto* buffer2 = new uint8_t[sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer)];
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::SharedMap3d, (void*)buffer, sizeof(map::voxel::shared_map_t));
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::ProcessedGridMap, (void*)buffer2, sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer));
    
// #if ENABLE_MASK
//     rock_perceptor_enable_mask(true);
// #else
//     rock_perceptor_enable_mask(false);
// #endif

    while (auto data = log_parser.get_data()) {
        auto& raw = data->get();

        if (!time_reset) {
            TestTimestamp::reset(raw.timestamp);
            rock_perceptor_set_chrono(TestTimestamp::now);
            time_reset = true;
        }

        if (!paused) {
            auto wait_time = (raw.timestamp - TestTimestamp::now()) * 1000;

            if (wait_time > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(wait_time)));
            } else {
                TestTimestamp::reset(raw.timestamp);
            }
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 1) {
            log::DEBUG_STREAM("Cmd_Pause true timestamp: " << raw.timestamp);
            paused = true;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = true; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 0) {
            log::DEBUG_STREAM("Cmd_Pause false timestamp: " << raw.timestamp);
            paused = false;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = false; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::SlamPose && raw.sensor == 0) {
            log::DEBUG_STREAM("Receive slam pose: " << raw.timestamp);
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::SlamPose,
                                                            sizeof(pose_2d_stamped_t), [&](msg_t* ptr) {
                                                                auto& pose     = *(pose_2d_stamped_t*)ptr->data;
                                                                pose.timestamp = raw.timestamp;
                                                                pose.pose.point.x.metre = raw.position_2d.pos.x;
                                                                pose.pose.point.y.metre = raw.position_2d.pos.y;
                                                                pose.pose.theta         = raw.position_2d.pos.theta;
                                                            }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Tof && raw.tof.exposure == rock::log_parser::Tof::Exposure::HdrFlood) {
            auto tof = ObservationConverter::convert_to_tof(raw);
            log::DEBUG_STREAM("Receive tof: " << tof->timestamp << ", sensor: " << tof->sensor
                                              << ", type: " << tof->type);

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::Tof, sizeof(point_cloud_data_t) + sizeof(point_4d_raw_t) * tof->size,
                [&](msg_t* ptr) {
                    auto& observation = *(point_cloud_data_t*)(ptr->data);
                    observation       = *tof;
                    std::copy(tof->points_4d, tof->points_4d + tof->size,
                              observation.points_4d);
                }));

            ++count;
        }

#if ENABLE_MASK

        if (raw.type == rock::log_parser::LogDataType::Camera_Image) {
            rgb_img               = cv::imread(raw.camera.name, cv::IMREAD_COLOR);
            last_camera_timestamp = raw.timestamp;
            log::DEBUG_STREAM("[Wire] image timestamp = " << raw.timestamp);
            cv::imshow("rgb", rgb_img);
            cv::waitKey(10);
        }

        if (raw.type == rock::log_parser::LogDataType::WireMask) {
            auto mask_encode = raw.mask.mask;
            mask.clear();

            for (auto i = 0; i < mask_encode.size() / 2; ++i) {
                auto value_idx = 2 * i;
                auto count_idx = 2 * i + 1;

                for (auto j = 0; j < mask_encode[count_idx]; ++j) {
                    mask.push_back(static_cast<uint8_t>(mask_encode[value_idx]));
                }

            }

            cv::resize(rgb_img, rgb_img, cv::Size(640, 480));

            for (auto i = 0; i < rgb_img.rows; ++i) {
                for (auto j = 0; j < rgb_img.cols; ++j) {
                    auto it    = rgb_img.cols * i + j;
                    auto label = mask[it];

                    if (label == 0) {
                        continue;
                    }

                    cv::Vec3b point             = rgb_img.at<cv::Vec3b>(i, j);
                    cv::Vec3b color             = alpha * cv::Vec3b(0, 0, 255) + (1 - alpha) * point;
                    rgb_img.at<cv::Vec3b>(i, j) = color;
                }
            }

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::MLMask, sizeof(rock_seg_t) + sizeof(uint8_t) * rgb_img.rows * rgb_img.cols,
                [&](msg_t* ptr) {
                    auto& seg     = *(rock_seg_t*)(ptr->data);
                    seg.timestamp = static_cast<double>(raw.timestamp);
                    seg.size      = rgb_img.rows * rgb_img.cols;
                    memcpy(seg.mask, mask.data(), sizeof(uint8_t) * rgb_img.rows * rgb_img.cols);
                }));

            cv::imshow("rgb", rgb_img);
            cv::waitKey(10);
        }

#else

        if (raw.type == rock::log_parser::LogDataType::Camera_Image) {
            rgb_img               = cv::imread(raw.camera.name, cv::IMREAD_COLOR);
            last_camera_timestamp = raw.timestamp;
            log::DEBUG_STREAM("[Wire] image timestamp = " << raw.timestamp);
            cv::imshow("rgb", rgb_img);
            cv::waitKey(10);
        }

        if (raw.type == rock::log_parser::LogDataType::WireRect) {
            line_points.clear();

            auto rects = raw.rect;

            std::vector<rect_points_t> cone_rects;

            for (auto i = 0; i < rects.points.size(); i += 4) {
                rect_points_t rect;

                for (auto j = 0; j < 4; ++j) {
                    rect.vertexes.point[j].x.metre = rects.points[i + j].x;
                    rect.vertexes.point[j].y.metre = rects.points[i + j].y;

                    auto p1 = cv::Point2f(rects.points[i + j].x * 800 / 640, rects.points[i + j].y * 600 / 480);
                    auto p2 = cv::Point2f(rects.points[(i + j + 1) % 4].x * 800 / 640,
                                          rects.points[(i + j + 1) % 4].y * 600 / 480);

                    cv::line(rgb_img, p1, p2, cv::Scalar(0, 255, 0), 2);

                    line_points.emplace_back(p1);
                    line_points.emplace_back(p2);
                }

                cone_rects.emplace_back(rect);
            }

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::MLRect,
                sizeof(rects_data_t) + sizeof(rect_points_t) * cone_rects.size(), [&](msg_t* ptr) {
                    auto& rr_rects     = *(rects_data_t*)(ptr->data);
                    rr_rects.timestamp = last_camera_timestamp;
                    rr_rects.size      = cone_rects.size();
                    log::DEBUG_STREAM("rect = (" << rr_rects.rects << ", " << &rr_rects.rects[0].rect << ", "
                                                 << &rr_rects.rects[0].vertexes << ")");
                    std::copy(cone_rects.begin(), cone_rects.end(), rr_rects.rects);
                }));

            cv::imshow("rgb", rgb_img);
            cv::waitKey(10);
        }

#endif
        rock::perceptor::util::Timer::PrintAll();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(1000)));
    rock_perceptor_controller_finish();
    return 0;
}
