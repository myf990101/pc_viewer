//
// Created by ray on 2024/2/8.
//

#pragma once
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

#include "Base.hpp"
#include "utils/Utils.h"

namespace rock::log_replay::ros_wrapper {
    auto constexpr RobotHeight = 0.085f;
    class Pose : public SensorBase {
    public:
        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            msg_header_.seq      = 0;
            msg_header_.frame_id = "map";
            path_pub_            = nh.advertise<nav_msgs::Path>("/trajectory", 100);
            data_pub_            = nh.advertise<geometry_msgs::PoseStamped>("/pose", 100);
            footprint_pub_       = nh.advertise<geometry_msgs::PolygonStamped>("/footprint", 100);

            cleaner_model_pub_   = nh.advertise<visualization_msgs::Marker>("/cleaner", 100);

            cleaner_model_.ns      = "cleaner";
            cleaner_model_.id      = 1;
            cleaner_model_.action  = visualization_msgs::Marker::ADD;
            cleaner_model_.type    = visualization_msgs::Marker::CYLINDER;
            cleaner_model_.scale.x = RobotRadius * 2;
            cleaner_model_.scale.y = RobotRadius * 2;
            cleaner_model_.scale.z = RobotHeight;
            cleaner_model_.color.r = 0.f;
            cleaner_model_.color.g = 1.f;
            cleaner_model_.color.b = 0.f;
            cleaner_model_.color.a = 0.8f;

            for (auto i = 0; i < 360; i += 10) {
                geometry_msgs::Point32 p;
                p.x = RobotRadius * std::cos(i * Deg2Rad);
                p.y = RobotRadius * std::sin(i * Deg2Rad);
                footprint_.points.emplace_back(p);
            }
        }

        void process(sensor_data_t& data) {
            ros::Time time;
            time.fromSec(data.timestamp);

            ++msg_header_.seq;
            msg_header_.stamp = time;

            auto               x = data.slam_pose.current.x;
            auto               y = data.slam_pose.current.y;
            Eigen::Quaternionf q = rock::log_parser::euler_to_quaternion(data.slam_pose.current.theta);

            // publish tf
            static auto br = tf::TransformBroadcaster();
            br.sendTransform(
                tf::StampedTransform(tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(x, y, 0)),
                                     time, "map", "base_link"));

            // publish pose
            geometry_msgs::PoseStamped pose;
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.header          = msg_header_;
            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(pose);
            }

            // publish footprint
            msg_footprint_.header = msg_header_;
            msg_footprint_.polygon.points.clear();
            update_footprint(pose, msg_footprint_);
            if (footprint_pub_.getNumSubscribers()) {
                footprint_pub_.publish(msg_footprint_);
            }

            // publish path
            if (msg_path_.poses.empty()) {
                msg_path_.poses.emplace_back(pose);
                return;
            }

            auto last_pose = msg_path_.poses.back();
            if ((std::abs(last_pose.pose.position.x - x) > 0.01 || std::abs(last_pose.pose.position.y - y) > 0.01)
                && time - last_pose.header.stamp > ros::Duration(1)) {
                msg_path_.poses.emplace_back(pose);
            }

            if (path_pub_.getNumSubscribers()) {
                msg_path_.header = msg_header_;
                path_pub_.publish(msg_path_);
            }

            // publish cleaner model
            cleaner_model_.header          = msg_header_;
            cleaner_model_.pose            = pose.pose;
            cleaner_model_.pose.position.z = RobotHeight / 2.f;
            cleaner_model_pub_.publish(cleaner_model_);
        }

    private:
        void update_footprint(geometry_msgs::PoseStamped& pose, geometry_msgs::PolygonStamped& footprint) {
            for (auto& p : footprint_.points) {
                geometry_msgs::Point32 pt;
                pt.x = pose.pose.position.x + p.x;
                pt.y = pose.pose.position.y + p.y;
                footprint.polygon.points.emplace_back(pt);
            }
        }

    private:
        std_msgs::Header msg_header_;

        nav_msgs::Path msg_path_;
        ros::Publisher path_pub_;

        ros::Publisher                footprint_pub_;
        geometry_msgs::Polygon        footprint_;
        geometry_msgs::PolygonStamped msg_footprint_;

        ros::Publisher cleaner_model_pub_;
        visualization_msgs::Marker cleaner_model_;
    };
}    // namespace rock::log_replay::ros_wrapper