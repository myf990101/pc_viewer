//
// Created by ray on 2024/2/8.
//

#pragma once
#include "type/log_type.hpp"

namespace rock::log_replay {
    using log_type_t = rock::log_parser::LogType;
    using log_path_t = rock::log_parser::LogPath;

    static auto NavPath                  = log_path_t{log_type_t::Nav, "/NAV_binId9*"};
    static auto MapPath                  = log_path_t{log_type_t::Map, "/user_map0*"};
    static auto IconPath                 = log_path_t{log_type_t::Icon, "/RRLDR_PERSIST_normal*"};
    static auto ImagePath                = log_path_t{log_type_t::Camera, "/*_IR_*"};
    static auto CommandPath              = log_path_t{log_type_t::Command, "/SLAM_fprintf*"};
    static auto WireRectPath             = log_path_t{log_type_t::WireRect, "/PERCEPTOR_normal*"};
    static auto ObjRectPath              = log_path_t{log_type_t::ObjRect, "/PERCEPTOR_normal*"};
    static auto WireMaskPath             = log_path_t{log_type_t::WireMask, "/wire.mask"};
    static auto AvoidCubePath            = log_path_t{log_type_t::AvoidCube, "/RRLDR_SERVER_normal*"};
    static auto SensorLogPath            = log_path_t{log_type_t::RockSensor, "/RRLDR_fprintf*"};
    static auto PointCloudDownSamplePath = log_path_t{log_type_t::Tof, "/NAV_binId10*"};
    static auto PointCloudSourcePath     = log_path_t{log_type_t::Tof, "/NAV_binId14*"};

}    // namespace rock::log_replay
