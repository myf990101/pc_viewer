#include <cstdint>
#include <yaml-cpp/yaml.h>

// #include "global_definition.h"
#include "interface/log_file_definition.hpp"
#include "interface/observation_converter.hpp"
#include "log_parser.hpp"
#include "mapping3d/TofMapper.h"
#include "mapprocess/MapWall.hpp"
#include "perceptor.h"
#include "util/Log.h"

extern std::string test_config;

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
        case RockPerceptorMessageType::UpdatedRectMapInfo: {
            auto& total   = *(rect_3d_t*)message->data;
            auto& updated = *(rect_3d_t*)(message->data + sizeof(rect_3d_t));
            log::DEBUG("[Callback] map update total = (%d, %d, %d, %d, %d, %d)", total.point.x.grid, total.point.y.grid,
                       total.point.z, total.size.length, total.size.width, total.size.height);
            log::DEBUG("[Callback] map update updated = (%d, %d, %d, %d, %d, %d)", updated.point.x.grid,
                       updated.point.y.grid, updated.point.z, updated.size.length, updated.size.width, updated.size.height);
        }
        default:
            break;
    }
}

int main(int argc, char** argv) {
    using namespace rock;
    using namespace rock::perceptor;
    using namespace rock::log_replay;
    using namespace rock::perceptor::common;
    using namespace rock::perceptor::map;

    auto conf          = YAML::LoadFile(WORK_SPACE_LOCATION"/config/mapping3d.yaml");
    auto log_directory = conf["replay"]["log_directory"].as<std::string>();
    auto start_time    = conf["replay"]["log_start"].as<timestamp_t>();
    test_config = WORK_SPACE_LOCATION"/config/mapping3d.yaml";

    if (log_directory.empty()) {
        log::ERROR("No valid log direction found, exit.");
        exit(0);
    }
    auto log_parser = rock::log_parser::LogParser(
        {{.type = PointCloudSourcePath.type, .path = log_directory + PointCloudSourcePath.path},
         {.type = CommandPath.type, .path = log_directory + CommandPath.path},
         {.type = SensorLogPath.type, .path = log_directory + SensorLogPath.path}});
    log_parser.set_log_time(start_time);

    auto* buffer = new uint8_t[sizeof(voxel::shared_map_t)];
    log::DEBUG("shared cell = %d, shared map = %d", sizeof(voxel::shared_cell_t), sizeof(voxel::shared_map_t));
    rock_perceptor_controller_init(callback);
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::SharedMap3d, (void*)buffer, sizeof(map::voxel::shared_map_t));

    auto* buffer2 = new uint8_t[sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer)];
    cv::Mat map2dchannel = cv::imread(conf["wall"]["map_2d_file"].as<std::string>(), 0);

    if (map2dchannel.empty()) {
        map2dchannel =   cv::Mat::zeros(1024, 1024, CV_8UC1);
    }

    cv::Mat map_for_process = cv::Mat::zeros(1024, 1024, CV_8UC1);
    for (auto i = 0; i < 1024; ++i) {
        for (auto j = 0; j < 1024; ++j) {
            if (map2dchannel.at<uchar>(j, i) > 100 && map2dchannel.at<uchar>(j, i) < 150) {
                continue;
            } else if (map2dchannel.at<uchar>(j, i) < 10) {
                map_for_process.at<uchar>(j, i) = 1;
            } else if (map2dchannel.at<uchar>(j, i) > 250) {
                map_for_process.at<uchar>(j, i) = 255;
            }
        }
    }
    map2dchannel = map_for_process;
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::ProcessedGridMap, (void*)buffer2, sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer));

    auto count      = 0u;
    auto paused     = false;
    auto time_reset = false;

    for (auto& raw : log_parser) {
        if (raw.timestamp < start_time) {
            continue;
        }

        if (!time_reset) {
            TestTimestamp::reset(raw.timestamp);
            rock_perceptor_set_chrono(TestTimestamp::now);
            time_reset = true;
        }

        if (!paused) {
            auto wait_time = (raw.timestamp - TestTimestamp::now()) * 1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(wait_time)));
        }else{
            TestTimestamp::reset(raw.timestamp);
            rock_perceptor_set_chrono(TestTimestamp::now);
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 1) {
            log::DEBUG("Cmd_Pause true timestamp: %f", raw.timestamp);
            paused = true;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = true; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 0) {
            log::DEBUG("Cmd_Pause false timestamp: %f", raw.timestamp);
            paused = false;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = false; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_SaveMap && raw.sensor == 0) {
            log::DEBUG("Cmd_SaveMap %d timestamp: %f", raw.command.map_id, raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::SaveMap, sizeof(int32_t),
                             [&](msg_t* ptr) { *(bool*)ptr->data = raw.command.map_id; }));
            using map_t = voxel::shared_map_t;

            auto save_map_file = [&](std::string const& file_name, map_t* map) {
                FILE* fp = fopen(file_name.c_str(), "wb");

                if (!fp) {
                    log::WARN("File open failed!");
                    return;
                }

                fwrite(map, sizeof(map_t), 1, fp);
                fclose(fp);
            };

            auto* map = (map_t*)buffer;
            save_map_file(log_directory + "/maps/user_map" + std::to_string(raw.command.map_id), map);
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Load && raw.sensor == 0) {
            log::DEBUG("Cmd_Load %d timestamp: %f", raw.command.map_id, raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::LoadMap, sizeof(int32_t),
                             [&](msg_t* ptr) { *(bool*)ptr->data = raw.command.map_id; }));
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_RotateMap && raw.sensor == 0) {
            log::DEBUG("Cmd_RotateMap timestamp: %f", raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::RotateMap, sizeof(pose_t), [&](msg_t* ptr) {
                    auto& pose         = *(pose_t*)ptr->data;
                    pose.point.x.metre = raw.command.pose.x;
                    pose.point.y.metre = raw.command.pose.y;
                    pose.theta         = raw.command.pose.theta;
                }));
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Lock) {
            log::DEBUG("Cmd_Lock %s timestamp: %f", static_cast<bool>(raw.sensor) ? "true" : "false", raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::LockMap, sizeof(bool),
                             [&raw](msg_t* ptr) {
                                 *(bool*) ptr->data = static_cast<bool>(raw.sensor);
                             }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Reset) {
            log::DEBUG("Cmd Reset time is : %f", raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::Reset, {}, [](msg_t* ptr) { return; }));
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_SetPose) {
            log::DEBUG("Cmd SetPose time is : %f", raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::SetPose, sizeof(pose_t), [&](msg_t* ptr) {
                    auto& pose         = *(pose_t*)ptr->data;
                    pose.point.x.metre = raw.command.pose.x;
                    pose.point.y.metre = raw.command.pose.y;
                    pose.theta         = raw.command.pose.theta;
                }));
        }

        if (paused) {
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::SlamPose) {
        //            log::DEBUG("Recive slam pose: %lf", raw.timestamp);
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

        if (raw.type == rock::log_parser::LogDataType::Tof) {
            auto tof = ObservationConverter::convert_to_tof(raw);
            log::DEBUG("Recive tof: %lf, sensor: %d, type: %d", tof->timestamp, tof->sensor, tof->type);

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::Tof, sizeof(point_cloud_data_t) + sizeof(point_5d_raw_t) * tof->size,
                [&](msg_t* ptr) {
                    auto& observation = *(point_cloud_data_t*)(ptr->data);
                    observation       = *tof;
                    std::copy(tof->points_5d, tof->points_5d + tof->size,
                              observation.points_5d);
                }));

            ++count;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(1000)));
    rock_perceptor_controller_finish();
    return 0;
}
