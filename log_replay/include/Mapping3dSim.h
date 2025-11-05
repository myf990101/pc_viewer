//
// Created by gaoyuan on 2024/7/22.
//

#pragma once

#include "interface/ros_interface.hpp"
#include "mapping3d/TofMapper.h"
#include "map/voxel/IVoxelSearch.h"

namespace rock::log_replay {
    class Mapping3dSim : public RosInterface {
    public:
        using mapper_t             = perceptor::mapping3d::TofMapper;
        using mapper_ptr_t         = std::unique_ptr<mapper_t>;
        using map_key_t            = mapper_t::ivoxel_map_t::key_t;
        using map_points_indexes_t = std::unordered_map<map_key_t, size_t>;
        using point_t              = mapper_t::point_t;
        using point_cloud_t        = mapper_t::point_cloud_t;
        using point_cloud_ptr_t    = point_cloud_t::ptr_t;
        using rect_3d_t            = perceptor::common::Rectangle3D;
        using rects_3d_t           = std::vector<rect_3d_t>;

    public:
        Mapping3dSim();

        ~Mapping3dSim() = default;

        void init() override;

        void run() override;

        template<uint32_t Scale_t>
        auto vectorize(point_3d_grid_t key) const -> point_3d_metre_t;

        void publish_map_points();

        void publish_map_3d(void* buffer, bool total = false);

    private:
        mapper_ptr_t         mapper_;
        size_t               map_counter_;
        point_cloud_ptr_t    map_points_cache_;
        point_cloud_ptr_t    map_points_cache_buffer_;
        map_points_indexes_t map_points_indexes_;
        point_cloud_ptr_t    shared_points_cache_;
        map_points_indexes_t shared_points_indexes_;

        ros_wrapper::Map        map_wrapper_;
        ros_wrapper::Pose       pose_wrapper_;
        ros_wrapper::StaticMap  static_map_wrapper_;
        ros_wrapper::PointCloud spot_wrapper_;
        ros_wrapper::PointCloud flood_wrapper_;
        ros_wrapper::PointCloud cloud_map_wrapper_;
        ros_wrapper::Marker     marker_wrapper_;
        ros_wrapper::PointCloud shared_map_wrapper_;
        ros_wrapper::Marker     timestamp_wrapper_;
    };
}

