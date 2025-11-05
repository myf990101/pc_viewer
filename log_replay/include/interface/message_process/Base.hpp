//
// Created by ray on 2024/2/26.
//

#pragma once
#include <ros/ros.h>

#include "type/log_type.hpp"
#include "type/tof_log_type.hpp"

namespace rock::log_replay::ros_wrapper {
    constexpr auto Deg2Rad     = float(M_PI / 180);
    constexpr auto RobotRadius = float(0.1725);

    class SensorBase {
    public:
        using sensor_data_t = rock::log_parser::LogData;

    public:
        virtual void init(ros::NodeHandle&, const std::string&, const std::string& = "") = 0;

        virtual void process(sensor_data_t&) = 0;

    protected:
        ros::Publisher data_pub_;
    };
}    // namespace rock::log_replay::ros_wrapper