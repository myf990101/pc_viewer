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
#include "common/PointCloud.h"
#include "common/Types.h"
#include "perceptor_msg_types.h"

namespace rock::log_replay::ros_wrapper {
    class PointCloud : public SensorBase {
    public:
        using sint_t = int32_t;

        using pcl_point_t      = pcl::PointXYZINormal;
        using pcl_pointcloud_t = pcl::PointCloud<pcl_point_t>;

        using pointcloud_4d
            = rock::perceptor::common::PointCloud<rock::perceptor::common::PointXYZIType<metre_t>>;
        using point_4d_metre_cloud
            = rock::perceptor::common::PointCloud<rock::perceptor::common::PointXYZINormCurv<metre_t>>;

    public:
        PointCloud() : msg_cloud_{new pcl_pointcloud_t()} {}

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            msg_header_.seq    = 0;
            msg_cloud_->height = 1;
            if (frame == "") {
                msg_header_.frame_id = "base_link";
            } else {
                msg_header_.frame_id = frame;
            }

            data_pub_ = nh.advertise<sensor_msgs::PointCloud2>(name, 100);
        }

        // for log replay
        void process(sensor_data_t& data) override {
            msg_cloud_->width = data.tof.size;
            msg_cloud_->resize(msg_cloud_->width);

            for (auto i = 0; i < msg_cloud_->size(); ++i) {
                msg_cloud_->points[i].x         = data.tof.cloud[i].x;
                msg_cloud_->points[i].y         = data.tof.cloud[i].y;
                msg_cloud_->points[i].z         = data.tof.cloud[i].z;
                msg_cloud_->points[i].normal_z  = data.tof.cloud[i].amp_con.confidence;
                msg_cloud_->points[i].intensity = to_rviz_color(data.tof.cloud[i].i);
                msg_cloud_->points[i].curvature = data.tof.cloud[i].index;
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

        void process_tof_raw(const sensor_data_t& data) {
            msg_cloud_->height = 1;
            msg_cloud_->width  = data.tof.size;
            msg_cloud_->resize(data.tof.size);

            for (auto i = 0; i < msg_cloud_->size(); ++i) {
                msg_cloud_->points[i].x         = data.tof.cloud[i].x;
                msg_cloud_->points[i].y         = data.tof.cloud[i].y;
                msg_cloud_->points[i].z         = data.tof.cloud[i].z;
                msg_cloud_->points[i].intensity = data.tof.cloud[i].i;
                msg_cloud_->points[i].curvature = data.tof.cloud[i].flag;
            }

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec(data.timestamp);
            ++msg_header_.seq;
            cloud.header = msg_header_;
            data_pub_.publish(cloud);
        }

        // viz tofmerger's output
        void process(multi_scan_t& data, bool only_laser4 = false) {
            const auto& multi_scan = data;
            auto*       scans_ptr  = (uint8_t*)data.scans;
            msg_cloud_->clear();

            uint64_t offset = 0;
            for (auto i = 0; i < multi_scan.scans_count; ++i) {
                auto* scans = (scan_t*)(scans_ptr + offset);
                for (auto k = 0; k < scans->count; ++k) {
                    const auto& beam = scans->beams[k];

                    pcl_point_t pcl_point;
                    pcl_point.x = beam.polar.rho * std::cos(beam.polar.theta);
                    pcl_point.y = beam.polar.rho * std::sin(beam.polar.theta);
                    pcl_point.z = 0;

                    if (only_laser4) {
                        if (i == 4) {
                            pcl_point.intensity = beam.flag * 20;
                            msg_cloud_->points.push_back(pcl_point);
                        }
                    } else {
                        msg_cloud_->points.push_back(pcl_point);
                    }
                }

                offset += (scans->count * sizeof(beam_t) + sizeof(scan_t));
            }

            msg_cloud_->height = 1;
            msg_cloud_->width  = msg_cloud_->points.size();

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec(data.timestamp);
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (only_laser4) {
                ROS_INFO("laser4 size : %zu", msg_cloud_->points.size());
            }

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
            }
        }

        //for classify_sim
        void process(pointcloud_4d::ptr_t data) {
            msg_cloud_->clear();
            msg_cloud_->width = data->size();
            msg_cloud_->resize(msg_cloud_->width);

            for (auto i = 0; i < msg_cloud_->size(); ++i) {
                if ((*data)[i].type() == static_cast<int>(ClassifyType::Noise)) {
                    continue;
                }

                msg_cloud_->points[i].x = (*data).points()[i].x();
                msg_cloud_->points[i].y = (*data).points()[i].y();
                msg_cloud_->points[i].z = (*data).points()[i].z();
                msg_cloud_->points[i].normal_z  = (*data).points()[i].correct_z();
                msg_cloud_->points[i].intensity = to_rviz_color((*data).points()[i].type());
                msg_cloud_->points[i].curvature = (*data).points()[i].id();
            }

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec((*data).timestamp());
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
            }
        }

        template<typename XYZI>
        void filter_process(const std::vector<XYZI>& pts, double t) {
            msg_cloud_->clear();
            for (const auto& p : pts) {
                pcl_point_t point;
                point.x         = p.x;
                point.y         = p.y;
                point.z         = p.z;
                point.intensity = p.i;
                msg_cloud_->points.push_back(point);
            }

            msg_cloud_->height = 1;
            msg_cloud_->width  = msg_cloud_->points.size();

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec(t);
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
            }
        }

        //for mapping3d
        void process(point_4d_metre_cloud::ptr_t const& data, bool curvature = false) {
            msg_cloud_->width = data->size();
            msg_cloud_->resize(msg_cloud_->width);

            for (auto i = 0; i < msg_cloud_->size(); ++i) {
                msg_cloud_->points.at(i).getVector3fMap() = data->at(i).map();
                msg_cloud_->points[i].intensity = curvature ? data->at(i).curvature() : data->at(i).intensity();
            }

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec((*data).timestamp());
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
                ROS_INFO_STREAM("[stable size] publish size" << cloud.width);
            }
        }

        template<typename PointT>
        void process(PointT& data, double t) {
            msg_cloud_->clear();
            for (const auto& p : data) {
                pcl_point_t point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                msg_cloud_->points.push_back(point);
            }

            msg_cloud_->height = 1;
            msg_cloud_->width  = msg_cloud_->points.size();

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec(t);
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
            }
        }

        template<typename PointT>
        void process_has_intensity(const PointT& data, double t) {
            msg_cloud_->clear();
            for (const auto& p : data) {
                pcl_point_t point;
                point.x         = p.x;
                point.y         = p.y;
                point.z         = p.z;
                point.intensity = p.i;
                msg_cloud_->points.push_back(point);
            }
            msg_cloud_->height = 1;
            msg_cloud_->width  = msg_cloud_->points.size();

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*msg_cloud_, cloud);
            msg_header_.stamp.fromSec(t);
            ++msg_header_.seq;
            cloud.header = msg_header_;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(cloud);
            }
        }

    private:
        static auto get_type_from_intensity(const sint_t& intensity) -> ClassifyColor {
            switch (intensity) {
                case 0:
                    return ClassifyColor::Noise;
                case 1:
                    return ClassifyColor::Ground;
                case 2:
                    return ClassifyColor::Obstacle;
                case 3:
                    return ClassifyColor::Wall;
                case 4:
                    return ClassifyColor::LooseWire;
                case 5:
                    return ClassifyColor::CoiledWire;
                case 6:
                    return ClassifyColor::Unknown;
                case 9:
                    return ClassifyColor::FakeWirePoint;
                default:
                    return ClassifyColor::Unknown;
            }
        }

        static auto to_rviz_color(const sint_t& intensity) -> sint_t {
            return static_cast<sint_t>(get_type_from_intensity(intensity));
        }

    private:
        std_msgs::Header      msg_header_;
        pcl_pointcloud_t::Ptr msg_cloud_;
    };
}    // namespace rock::log_replay::ros_wrapper
