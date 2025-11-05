//
// Created by ray on 2024/2/26.
//

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    class Cliff : public SensorBase {
    public:
        using pcl_point_t      = pcl::PointXYZ;
        using pcl_pointcloud_t = pcl::PointCloud<pcl_point_t>;

        enum CliffSensor { FL = 0, FR, CL, CR, BL, BR };

        std::vector<int> CliffAngle{345, 15, 300, 60, 270, 90};

    public:
        Cliff() : msg_cloud_{new pcl_pointcloud_t()} {}

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") {
            msg_header_.seq      = 0;
            msg_cloud_->height   = 1;
            msg_header_.frame_id = "base_link";

            data_pub_ = nh.advertise<sensor_msgs::PointCloud2>(name, 100);

            for (const auto& angle : CliffAngle) {
                Eigen::Vector3f p;
                p.x() = RobotRadius * std::cos(angle * Deg2Rad);
                p.y() = RobotRadius * std::sin(angle * Deg2Rad);
                cliff_position_.emplace_back(p);
            }
        }

        void process(sensor_data_t& data) override {
            msg_cloud_->points.clear();

            if (data.cliff.FL) {
                msg_cloud_->points.emplace_back(get_cliff(CliffSensor::FL));
            }

            if (data.cliff.FR) {
                msg_cloud_->points.emplace_back(get_cliff(CliffSensor::FR));
            }

            if (data.cliff.CL) {
                msg_cloud_->points.emplace_back(get_cliff(CliffSensor::CL));
            }

            if (data.cliff.CR) {
                msg_cloud_->points.emplace_back(get_cliff(CliffSensor::CR));
            }

            if (data.cliff.BL) {
                msg_cloud_->points.emplace_back(get_cliff(CliffSensor::BL));
            }

            if (data.cliff.BR) {
                msg_cloud_->points.emplace_back(get_cliff(CliffSensor::BR));
            }

            msg_cloud_->width = msg_cloud_->size();

            sensor_msgs::PointCloud2 cliff_cloud;
            pcl::toROSMsg(*msg_cloud_, cliff_cloud);
            ++msg_header_.seq;
            msg_header_.stamp.fromSec(data.timestamp);
            cliff_cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cliff_cloud);
            }
        }

    private:
        auto get_cliff(CliffSensor id) -> pcl::PointXYZ {
            auto point = cliff_position_[id];
            return pcl::PointXYZ(point.x(), point.y(), 0);
        }

    private:
        std_msgs::Header             msg_header_;
        pcl_pointcloud_t::Ptr        msg_cloud_;
        std::vector<Eigen::Vector3f> cliff_position_;
    };
}    // namespace rock::log_replay::ros_wrapper