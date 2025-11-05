//
// Created by ray on 2024/3/18.
//

#pragma once
#include "perceptor_msg_types.h"
#include "type/log_type.hpp"
#include "type/tof_log_type.hpp"

namespace rock::log_replay {
    using sensor_data_t = rock::log_parser::LogData;
    class ObservationConverter {
    public:
        ObservationConverter() = default;

        static void* rock_slam_memory_allocate(uint32_t size) {
            return std::memset(malloc(size), {}, size);
        }

        struct Deleter {
            void operator()(point_cloud_data_t* ptr) {
                free(ptr);
            }
        };

        static auto convert_to_tof(sensor_data_t& raw) -> std::unique_ptr<point_cloud_data_t, Deleter> {
            auto tof = std::unique_ptr<point_cloud_data_t, Deleter>{(point_cloud_data_t*)rock_slam_memory_allocate(
                sizeof(point_cloud_data_t) + sizeof(point_5d_raw_t) * raw.tof.cloud.size())};

            tof->timestamp    = raw.timestamp;
            tof->sensor = raw.sensor;
            tof->type   = static_cast<int>(raw.tof.exposure);
            tof->size   = raw.tof.cloud.size();
            tof->step   = sizeof(point_5d_raw_t);
            for (int i = 0; i < tof->size; ++i) {
                tof->points_5d[i].x.metre = raw.tof.cloud[i].x;
                tof->points_5d[i].y.metre = raw.tof.cloud[i].y;
                tof->points_5d[i].z.metre = raw.tof.cloud[i].z;
                tof->points_5d[i].i.grid  = raw.tof.cloud[i].i;    //in 'perceptor plugins repo', we use grid
                tof->points_5d[i].f.grid  = raw.tof.cloud[i].flag;
            }

            return tof;
        }
    };
}    // namespace rock::log_replay