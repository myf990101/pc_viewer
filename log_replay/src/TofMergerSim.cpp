#include "TofMergerSim.h"

#include <cstdint>

namespace rock::log_replay {

    constexpr auto MultiLineNum = 5;

    void write_multilaser(const multi_scan_t& src, const perceptor::tof::TofMerger::tof_multi_line_param_ptr_t& param,
                          std::ostream& os) {
        std::cout << "write=========" << std::endl;
        int64_t time_ms = static_cast<int>(std::round(src.timestamp * 1000.0));
        os << time_ms << " "
           << "slamLaserMulti"
           << " " << src.id << " " << src.scans_count;

        const auto& multi = src;
        for (int i = 0; i < multi.scans_count; ++i) {
            os << " " << param->tof_to_2d_params[i].z_min << " " << param->tof_to_2d_params[i].z_max;
        }
        os << "\n";

        uint8_t* scans_ptr = (uint8_t*)src.scans;
        uint64_t offset    = 0;
        for (int i = 0; i < multi.scans_count; ++i) {
            scan_t* scans = (scan_t*)(scans_ptr + offset);
            std::cout << time_ms << " slamLaser" << i << " " << multi.id << " " << param->tof_to_2d_params[i].range_max
                      << " " << scans->count << std::endl;
            os << time_ms << " slamLaser" << i << " " << multi.id << " " << param->tof_to_2d_params[i].range_max << " "
               << scans->count;
            for (int k = 0; k < scans->count; ++k) {
                const auto& beam      = scans->beams[k];
                int         intensity = beam.intensity > 0 ? 1 : 0;    //for now the 'perceptor plugins repo' use 1 or 0
                os << " " << beam.polar.theta << " " << beam.polar.rho << " " << beam.flag;
            }
            os << "\n";
            offset += (scans->count * sizeof(beam_t) + sizeof(scan_t));
        }
    }

    TofMergerSim::TofMergerSim() : RosInterface() {}

    void TofMergerSim::init() {
        algorithm_ = std::make_unique<perceptor::tof::TofMerger>();

        uint32_t data_size = sizeof(tof_multi_line_param_t) + MultiLineNum * sizeof(tof_to_2d_param_t);

        multi_line_param_ptr
            = tof_multi_line_param_type{(tof_multi_line_param_t*)rock_perceptor_memory_allocate(data_size)};

        multi_line_param_ptr->line_num = MultiLineNum;
        auto* tof_to_2d_params         = multi_line_param_ptr->tof_to_2d_params;

        //change chaofeng or normal tof
        std::string hardware_type = get_param<std::string>("replay/hardware", "unknown");
        if (hardware_type == "oms") {
            multi_line_param_ptr->tof_hardware = TofHardware::OMS;
        } else if (hardware_type == "chaofeng") {
            multi_line_param_ptr->tof_hardware = TofHardware::ChaoFeng;
        } else if (hardware_type == "chaofeng_oms") {
            multi_line_param_ptr->tof_hardware = TofHardware::ChaoFengOMS;
        } else {
            ROS_FATAL("bad hardware type '%s', shoule be oms / chaofeng / chaofeng_oms", hardware_type.c_str());
        }

        float range_max                = 8.0;
        float range_min                = 0.07;
        float z_max                    = 0.00;
        float z_min                    = 0.00;
        auto  strategy                 = RockTofTo2dStrategy::MinDistance;
        bool  tof_filter               = false;
        bool  slam_enable_angle_filter = false;
        float angle_resolution_deg     = 1.0;
        auto  tof_2d_params            = get_param<XmlRpc::XmlRpcValue>("tof_to_2d_params", XmlRpc::XmlRpcValue{});
        for (auto i = 0; i < tof_2d_params.size(); i++) {
            auto& tof_2d_param = tof_2d_params[i];
            tof_to_2d_params[i].range_max
                = tof_2d_param.hasMember("range_max") ? static_cast<double>(tof_2d_param["range_max"]) : range_max;
            tof_to_2d_params[i].range_min
                = tof_2d_param.hasMember("range_min") ? static_cast<double>(tof_2d_param["range_min"]) : range_min;
            tof_to_2d_params[i].z_max
                = tof_2d_param.hasMember("z_max") ? static_cast<double>(tof_2d_param["z_max"]) : z_max;
            tof_to_2d_params[i].z_min
                = tof_2d_param.hasMember("z_min") ? static_cast<double>(tof_2d_param["z_min"]) : z_min;
            tof_to_2d_params[i].strategy = tof_2d_param.hasMember("strategy")
                                               ? (static_cast<std::string>(tof_2d_param["strategy"]) == "min_dis"
                                                      ? RockTofTo2dStrategy::MinDistance
                                                      : RockTofTo2dStrategy::AllFlat)
                                               : strategy;
            tof_to_2d_params[i].filter
                = tof_2d_param.hasMember("filter") ? static_cast<bool>(tof_2d_param["filter"]) : tof_filter;
            tof_to_2d_params[i].slam_enable_angle_filter
                = tof_2d_param.hasMember("slam_enable_angle_filter")
                      ? static_cast<bool>(tof_2d_param["slam_enable_angle_filter"])
                      : slam_enable_angle_filter;
            tof_to_2d_params[i].angle_resolution_deg = tof_2d_param.hasMember("angle_resolution_deg")
                                                           ? static_cast<double>(tof_2d_param["angle_resolution_deg"])
                                                           : angle_resolution_deg;
        }

        for (auto i = 0; i < MultiLineNum; i++) {
            ROS_DEBUG("tof_to_2d_params[%d], range: [%f, %f], z_range: [%f, %f], strategy: %d, filter: %d, "
                      "angle_filter: [%d, %f].",
                      i, tof_to_2d_params[i].range_min, tof_to_2d_params[i].range_max, tof_to_2d_params[i].z_min,
                      tof_to_2d_params[i].z_max, tof_to_2d_params[i].strategy, tof_to_2d_params[i].filter,
                      tof_to_2d_params[i].slam_enable_angle_filter, tof_to_2d_params[i].angle_resolution_deg);
        }

        algorithm_->set_tof_param(multi_line_param_ptr);

        pose_wrapper_.init(nh(), "/pose");
        left_wrapper_.init(nh(), "/left", "map");
        front_wrapper_.init(nh(), "/front", "map");
        merge_wrapper_.init(nh(), "/merge", "map");
        laser4_wrapper_.init(nh(), "/laser4", "map");
        valid_wrapper_.init(nh(), "/valid", "map");
        filter_wrapper_.init(nh(), "/filter", "map");
        points_classify_wrapper_.init(nh(), "/points_classify", "map");
        marker_wrapper_.init(nh(), "/time_display");

        std::vector<log_path_t> file_to_parse;
        get_file_list(file_to_parse);

        log_parser()->set_log_time(get_param("replay/log_start", 0), true);

        log_parser()->init(file_to_parse);

        log_parser()->begin();
    }

    void TofMergerSim::run() {
        ROS_INFO("begin ==============");

        auto last_observation_timestamp = double{-1};

        perceptor::tof::Debugger debugger;
        algorithm_->set_debugger(&debugger);

        bool          output_lidar_for_replace = get_param("replay/output_lidar_for_replace", false);
        std::ofstream ofs;
        if (output_lidar_for_replace) {
            std::string log_directory = get_param("replay/log_directory", std::string(""));
            std::string directory     = log_directory + "/replace_begin_end";
            std::string file_path     = directory + "/multi_laser.log";
            namespace fs              = std::filesystem;
            fs::path path(file_path);
            if (!fs::exists(path.parent_path())) {
                fs::create_directories(path.parent_path());
            }

            ofs.open(log_directory + "/replace_begin_end/multi_laser.log");
            ofs << "laserVersion 1" << std::endl;    // laserVersion 1 means intensity is flag
        }

        while (auto data = log_parser()->get_data()) {
            if (!ros::ok())    // natually die when caught Ctrl-C
                break;

            auto& raw = data->get();
            marker_wrapper_.process(raw);

            if (raw.type == log_data_type_t::Tof) {
                if (raw.tof.exposure != tof_t::Exposure::HdrSpot || raw.sensor == static_cast<int>(tof_t::Sensor::Top)
                    || raw.sensor == static_cast<int>(tof_t::Sensor::Unknown)) {
                    continue;
                }

                if (raw.sensor == static_cast<int>(tof_t::Sensor::Front)) {
                    front_wrapper_.process_tof_raw(raw);
                }

                if (raw.sensor == static_cast<int>(tof_t::Sensor::Left)) {
                    left_wrapper_.process_tof_raw(raw);
                }

                auto observation = ObservationConverter::convert_to_tof(raw);
                ROS_DEBUG("===== parse ===== t=%f sensor=%d exposure=%d", observation->timestamp, raw.sensor,
                          (int)raw.tof.exposure);

                if (last_observation_timestamp > 0 && raw.timestamp - last_observation_timestamp > 10) {
                    ROS_WARN("ToF msg time gap: %lf -> %lf", last_observation_timestamp, raw.timestamp);
                }

                algorithm_->update_pointcloud(*observation);
                last_observation_timestamp = raw.timestamp;

                auto* output = algorithm_->get_multi_laser();
                if (output) {
                    if (output_lidar_for_replace)
                        write_multilaser(*output, multi_line_param_ptr, ofs);

                    ROS_INFO("try to publish merge ======");
                    merge_wrapper_.process(*output);

                    ROS_INFO("try to publish laser4======");
                    laser4_wrapper_.process(*output, true);
                    // ========= debug =====//
                    if (debugger.valid.size()) {
                        ROS_INFO("pub  valid =-====");
                        valid_wrapper_.process(debugger.valid, raw.timestamp);
                    }
                    if (debugger.filtered.size()) {
                        ROS_INFO("pub  filtered =-====");
                        filter_wrapper_.filter_process(debugger.filtered, raw.timestamp);
                    }
                    if (debugger.points_classify.size()) {
                        ROS_INFO("pub  points_classify =-====");
                        points_classify_wrapper_.process_has_intensity(debugger.points_classify, raw.timestamp);
                    }
                    debugger.clear();
                }
            }

            if (raw.type == log_data_type_t::OdoGyro) {
                motion_t motion;
                motion.timestamp          = raw.timestamp;
                motion.pose.point.x.metre = raw.slam_pose.current.x;
                motion.pose.point.y.metre = raw.slam_pose.current.y;
                motion.pose.theta         = raw.slam_pose.current.theta;
                motion.gyro.euler.pitch   = raw.odo_gyro.euler.pitch;
                motion.gyro.euler.roll    = raw.odo_gyro.euler.roll;
                motion.gyro.euler.yaw     = raw.odo_gyro.euler.yaw;
                algorithm_->update_odom(motion);
                ROS_DEBUG("insert odom %.3f", motion.timestamp);
            }
        }

        ROS_INFO("end ==============");
    }
}    // namespace rock::log_replay
