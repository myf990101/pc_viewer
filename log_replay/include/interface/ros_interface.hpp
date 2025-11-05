//
// Created by ray on 2024/2/8.
//

#pragma once

#include <std_msgs/UInt8.h>

#include "log_file_definition.hpp"
#include "log_parser.hpp"
#include "message_process/Cliff.hpp"
#include "message_process/Icon.hpp"
#include "message_process/Image.hpp"
#include "message_process/Map.hpp"
#include "message_process/Marker.hpp"
#include "message_process/Nav.hpp"
#include "message_process/Polygon.hpp"
#include "message_process/Polygons.hpp"
#include "message_process/Pose.hpp"
#include "message_process/StaticMap.hpp"
#include "message_process/Tof.hpp"

namespace rock::log_replay {
    static constexpr auto DefaultLogRate    = float{0.2};
    static constexpr auto RateMinimumChange = float{0.01};
    static constexpr auto TimeMinimumChange = float{10};

    class RosInterface {
    public:
        using log_parser_t     = rock::log_parser::LogParser;
        using log_parser_ptr_t = std::unique_ptr<log_parser_t>;
        using log_data_type_t  = rock::log_parser::LogDataType;
        using tof_t            = rock::log_parser::Tof;
        using nav_t            = rock::log_parser::Nav;

        enum class KeyBoardCommand : uint8_t {
            KEYBOARD_PAUSE      = 0,
            KEYBOARD_RESUME     = 1,
            KEYBOARD_SPEED_UP   = 2,
            KEYBOARD_SPEED_DOWN = 3,
            KEYBOARD_ROLL_BACK  = 4,
            KEYBOARD_EXIT       = 10,
            KEYBOARD_REAL_TIME  = 11,
            KEYBOARD_MAX_SPEED  = 12,
        };

    public:
        RosInterface() : nh_(), pnh_("~"), log_parser_{new log_parser_t} {
            ROS_DEBUG("ROS Interface initialization finished");
            keyboard_sub_ = nh_.subscribe("/keypress_publisher/pressed_key", 10, &RosInterface::keyboard_command, this);
        }

        ~RosInterface() = default;

        virtual void init() = 0;

        virtual void run() = 0;

        auto nh() -> ros::NodeHandle& {
            return nh_;
        }

        auto log_parser() -> log_parser_ptr_t& {
            return log_parser_;
        }

        void keyboard_command(const std_msgs::UInt8::ConstPtr& msg) {
            static auto Rate = DefaultLogRate;

            switch (static_cast<KeyBoardCommand>(msg->data)) {
                case KeyBoardCommand::KEYBOARD_PAUSE:
                    log_parser_->pause();
                    ROS_DEBUG_STREAM("Pause");
                    break;
                case KeyBoardCommand::KEYBOARD_RESUME:
                    log_parser_->resume();
                    ROS_DEBUG_STREAM("Resume");
                    break;
                case KeyBoardCommand::KEYBOARD_SPEED_UP:
                    Rate -= RateMinimumChange;
                    log_parser_->set_rate(Rate);
                    ROS_DEBUG("Update speed, %f", Rate);
                    break;
                case KeyBoardCommand::KEYBOARD_SPEED_DOWN:
                    Rate += RateMinimumChange;
                    log_parser_->set_rate(Rate);
                    ROS_DEBUG("Down speed, %f", Rate);
                    break;
                case KeyBoardCommand::KEYBOARD_ROLL_BACK:
                    log_parser_->set_change_time(-TimeMinimumChange);
                    ROS_DEBUG("Roll back, %f", Rate);
                    break;
                case KeyBoardCommand::KEYBOARD_EXIT:
                    log_parser_->exit();
                    break;
                case KeyBoardCommand::KEYBOARD_REAL_TIME:
                    log_parser_->set_rate(Rate);
                    log_parser_->set_realtime(true);
                    ROS_DEBUG("Status: realtime: %d, rate %f\n", log_parser_->is_realtime(), log_parser_->get_rate());
                    break;
                case KeyBoardCommand::KEYBOARD_MAX_SPEED:
                    log_parser_->set_realtime(false);
                    ROS_DEBUG("Status: realtime: %d, rate %f\n", log_parser_->is_realtime(), log_parser_->get_rate());
                    break;
                default:
                    break;
            }
        }

    protected:
        template<typename T>
        auto get_param(const std::string& name, T default_val) -> T {
            T val;
            pnh_.param<T>(name, val, T());
            return val;
        }

        void get_file_list(std::vector<log_path_t>& file_to_parse) {
            std::string log_directory = get_param("replay/log_directory", std::string(""));
            if (log_directory.empty()) {
                ROS_ERROR("Set log direction and restart.");
                exit(0);
            }

            if (get_param("default/icon", true)) {
                file_to_parse.emplace_back(log_path_t{.type = IconPath.type, .path = log_directory + IconPath.path});
            }

            if (get_param("default/image", true)) {
                file_to_parse.emplace_back(log_path_t{.type = ImagePath.type, .path = log_directory + ImagePath.path});
            }

            if (get_param("default/user_map", true)) {
                file_to_parse.emplace_back(log_path_t{.type = MapPath.type, .path = log_directory + MapPath.path});
            }

            if (get_param("default/bin9", true)) {
                file_to_parse.emplace_back(log_path_t{.type = NavPath.type, .path = log_directory + NavPath.path});
            }

            if (get_param("default/bin10", false)) {
                file_to_parse.emplace_back(log_path_t{.type = PointCloudDownSamplePath.type,
                                                      .path = log_directory + PointCloudDownSamplePath.path});
            }

            if (get_param("default/bin14", false)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = PointCloudSourcePath.type, .path = log_directory + PointCloudSourcePath.path});
            }

            if (get_param("default/wire_rect", false)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = WireRectPath.type, .path = log_directory + WireRectPath.path});
            }

            if (get_param("default/obj_rect", false)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = ObjRectPath.type, .path = log_directory + ObjRectPath.path});
            }

            if (get_param("default/wire_mask", false)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = WireMaskPath.type, .path = log_directory + WireMaskPath.path});
            }

            if (get_param("default/slam_fprintf", true)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = CommandPath.type, .path = log_directory + CommandPath.path});
            }

            if (get_param("default/RRLDR_fprintf", true)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = SensorLogPath.type, .path = log_directory + SensorLogPath.path});
            }

            if (get_param("default/avoid_cube", true)) {
                file_to_parse.emplace_back(
                    log_path_t{.type = AvoidCubePath.type, .path = log_directory + AvoidCubePath.path});
            }
        }

    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber keyboard_sub_;

        log_parser_ptr_t log_parser_;
    };
}    // namespace rock::log_replay