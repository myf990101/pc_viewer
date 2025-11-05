//
// Created by ray on 2024/2/8.
//

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    class NavPoints : public SensorBase {
    public:
        using pcl_point_t      = pcl::PointXYZI;
        using pcl_pointcloud_t = pcl::PointCloud<pcl_point_t>;

    public:
        NavPoints() : msg_cloud_{new pcl_pointcloud_t()} {}

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            msg_header_.seq      = 0;
            msg_cloud_->height   = 1;
            msg_header_.frame_id = "/map";

            data_pub_ = nh.advertise<sensor_msgs::PointCloud2>(name, 100);
        }

        void process(sensor_data_t& data) override {
            msg_cloud_->width = data.nav.points.size();
            msg_cloud_->resize(msg_cloud_->width);

            for (auto i = 0; i < msg_cloud_->size(); ++i) {
                msg_cloud_->points[i].x = data.nav.points[i].x;
                msg_cloud_->points[i].y = data.nav.points[i].y;
                msg_cloud_->points[i].z = data.nav.points[i].z;
            }

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec(data.timestamp);
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
            }
        }

    private:
        std_msgs::Header      msg_header_;
        pcl_pointcloud_t::Ptr msg_cloud_;
    };
}    // namespace rock::log_replay::ros_wrapper