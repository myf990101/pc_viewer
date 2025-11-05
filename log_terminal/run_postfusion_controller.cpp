//
// Created by gaoyuan on 2023/9/11.
//
#include <yaml-cpp/yaml.h>

#include "detection/PostFusion.h"
// #include "global_definition.h"
#include <cstdint>

#include "interface/log_file_definition.hpp"
#include "interface/observation_converter.hpp"
#include "log_parser.hpp"
#include "mapping3d/TofMapper.h"
#include "mapprocess/MapWall.hpp"
#include "perceptor.h"
#include "util/Log.h"
#include "util/Timer.h"

extern std::string test_config;
static const auto  WaitForFutureTime = int32_t{50};    //ms

class TestTimestamp {
public:
    static double now() {
        auto duration = std::chrono::steady_clock::now() - instance().actual_start_;
        auto diff     = (double)duration.count() * decltype(duration)::period::num / decltype(duration)::period::den;
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

    double                                             test_start_;
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
        case RockPerceptorMessageType::PostFusionPoint: {
            auto observation = *(point_cloud_data_t**)message->data;
            log::INFO_STREAM("receive message... timestamp = " << observation->timestamp
                                                               << ", type = " << observation->type
                                                               << ", sensor = " << observation->sensor
                                                               << ", point num = " << observation->size);
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
    using namespace rock::perceptor::detection::post;
    using namespace rock::perceptor::map;

    auto conf          = YAML::LoadFile(WORK_SPACE_LOCATION "/config/post_fusion.yaml");
    auto log_directory = conf["replay"]["log_directory"].as<std::string>();

    test_config = std::string(WORK_SPACE_LOCATION "/config/post_fusion.yaml");

    auto count      = 0u;
    auto paused     = false;
    auto start_time = 743;
    auto alpha      = 0.5f;

    auto log_parser = rock::log_parser::LogParser(
        {{.type = PointCloudSourcePath.type, .path = log_directory + PointCloudSourcePath.path},
         {.type = CommandPath.type, .path = log_directory + CommandPath.path},
         {.type = SensorLogPath.type, .path = log_directory + SensorLogPath.path},
         {.type = ImagePath.type, .path = log_directory + ImagePath.path},
         {.type = ObjRectPath.type, .path = log_directory + ObjRectPath.path}});

    log_parser.set_log_time(start_time);

    log_parser.begin();

    auto time_reset = false;

    cv::Mat rgb_img;
    auto    last_camera_timestamp = timestamp_t{};
    auto*   buffer                = new uint8_t[1];

    std::vector<cv::Point2f> line_points;

    auto* buffer1 = new uint8_t[sizeof(voxel::shared_map_t)];
    auto* buffer2 = new uint8_t[sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer)];
    log::DEBUG("shared cell = %d, shared map = %d", sizeof(voxel::shared_cell_t), sizeof(voxel::shared_map_t));
    rock_perceptor_controller_init(callback);
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::SharedMap3d, (void*)buffer1, sizeof(map::voxel::shared_map_t));
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::ProcessedGridMap, (void*)buffer2, sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer));
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
            log::INFO_STREAM("Cmd_Pause true timestamp: " << raw.timestamp);
            paused = true;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = true; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 0) {
            log::INFO_STREAM("Cmd_Pause false timestamp: " << raw.timestamp);
            paused = false;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = false; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::SlamPose && raw.sensor == 0) {
            log::INFO_STREAM("Receive slam pose: " << raw.timestamp);
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

        if (raw.type == rock::log_parser::LogDataType::Tof
            && raw.tof.exposure == rock::log_parser::Tof::Exposure::HdrFlood) {
            auto tof = ObservationConverter::convert_to_tof(raw);
            log::INFO_STREAM("Receive tof: " << tof->timestamp << ", sensor: " << tof->sensor
                                             << ", type: " << tof->type);

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::Tof, sizeof(point_cloud_data_t) + sizeof(point_5d_raw_t) * tof->size,
                [&](msg_t* ptr) {
                    auto& observation = *(point_cloud_data_t*)(ptr->data);
                    observation       = *tof;
                    std::copy(tof->points_5d, tof->points_5d + tof->size,
                              observation.points_5d);
                }));

            ++count;
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::ObjRect) {
            line_points.clear();

            auto rects = raw.obj;

            std::vector<obj_point_t> cone_rects;

            for (auto i = 0; i < rects.obj.size(); i += 5) {
                obj_point_t rect;
                auto              type = rects.obj[i + 0];

                if (type != 8) {
                    continue;
                }

                rect.rect.point.x.metre = rects.obj[i + 1];
                rect.rect.point.y.metre = rects.obj[i + 2];
                rect.rect.size.width    = rects.obj[i + 3];
                rect.rect.size.height   = rects.obj[i + 4];
                rect.type               = type;

                auto p1 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre);
                auto p2 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width, rect.rect.point.y.metre);

                auto p3 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width, rect.rect.point.y.metre);
                auto p4 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width,
                                      rect.rect.point.y.metre + rect.rect.size.height);

                auto p5 = cv::Point2f(rect.rect.point.x.metre + rect.rect.size.width,
                                      rect.rect.point.y.metre + rect.rect.size.height);
                auto p6 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre + rect.rect.size.height);

                auto p7 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre + rect.rect.size.height);
                auto p8 = cv::Point2f(rect.rect.point.x.metre, rect.rect.point.y.metre);

                line_points.emplace_back(p1);
                line_points.emplace_back(p2);
                line_points.emplace_back(p3);
                line_points.emplace_back(p4);
                line_points.emplace_back(p5);
                line_points.emplace_back(p6);
                line_points.emplace_back(p7);
                line_points.emplace_back(p8);

                cone_rects.emplace_back(rect);
            }

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::MLObj, sizeof(objs_t) + sizeof(obj_point_t) * cone_rects.size(),
                [&](msg_t* ptr) {
                    auto& rr_rects     = *(objs_t*)(ptr->data);
                    rr_rects.timestamp = raw.timestamp;
                    rr_rects.size      = cone_rects.size();
                    std::cout << "rect send size:" << rr_rects.size << std::endl;
                    log::INFO_STREAM("rect = (" << rr_rects.rects << ", " << &rr_rects.rects[0].rect << ", "
                                                << ")");
                    std::copy(cone_rects.begin(), cone_rects.end(), rr_rects.rects);
                }));

            //            cv::imshow("rgb", rgb_img);
            cv::waitKey(10);
            continue;
        }

        //        rock::perceptor::util::Timer::PrintAll();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(1000)));
    rock_perceptor_controller_finish();
    return 0;
}
