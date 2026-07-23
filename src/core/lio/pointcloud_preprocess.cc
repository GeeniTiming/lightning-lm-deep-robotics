#include "pointcloud_preprocess.h"
#include <algorithm>
#include <cmath>
#include <execution>
#include <iomanip>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

namespace lightning {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

// Helper function: This function should be defined in a scope accessible by PointCloudPreprocess.
// It now only lists the types for which handlers exist in the switch statement.
std::string getSupportedLidarTypesAsString() {
    std::string types_list = "";
    types_list += "VELO32";
    types_list += ", OUST64";
    types_list += ", RoboSense";
    return types_list;
}

void PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudType::Ptr &pcl_out) {
    last_scan_start_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    switch (lidar_type_) {
        // Only include cases for the LidarType members that have corresponding handlers.
        // AVIA is intentionally excluded from here, so it will fall into the 'default' case.
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::RoboSense:
            RobosenseHandler(msg);
            break;
            
        default:
            // This 'default' case will catch AVIA and any other LidarType that doesn't have an explicit handler.
            LOG(ERROR) << "Error: Unknown or unsupported LiDAR Type encountered. "
                       << "Received numerical value: " << static_cast<int>(lidar_type_)
                       << ". Currently supported LiDAR types are: " << getSupportedLidarTypesAsString() << ".";
            break;
    }
    *pcl_out = cloud_out_;
}



void PointCloudPreprocess::Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg,
                                   PointCloudType::Ptr &pcl_out) {
    last_scan_start_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    cloud_out_.clear();
    cloud_full_.clear();

    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<char> is_valid_pt(plsize, 0);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;

                // use curvature as time of each laser points, curvature unit: ms
                cloud_full_[i].timestamp = msg->points[i].offset_time / double(1000000);

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                        (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                             cloud_full_[i].z * cloud_full_[i].z >
                         (blind_ * blind_))) {
                    is_valid_pt[i] = 1;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<PointType> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    
    double head_time = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        added_pt.timestamp = (pl_orig.points[i].timestamp - head_time) * 1e3;  //  / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.timestamp = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.timestamp = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.timestamp;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.timestamp = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.timestamp = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.timestamp < time_last[layer]) {
                added_pt.timestamp += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.timestamp;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}



void PointCloudPreprocess::RobosenseHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<robosense_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    const int plsize = pl_orig.points.size();
    if (plsize == 0) {
        return;
    }

    cloud_out_.reserve(plsize);
    pcl_conversions::toPCL(msg->header, cloud_out_.header);

    const double header_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    std::vector<std::size_t> finite_indices;
    finite_indices.reserve(plsize);
    for (std::size_t i = 0; i < pl_orig.points.size(); ++i) {
        const auto &pt = pl_orig.points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) ||
            !std::isfinite(pt.timestamp)) {
            continue;
        }
        finite_indices.emplace_back(i);
    }

    if (finite_indices.empty()) {
        LOG(WARNING) << "RoboSense scan has no finite points";
        return;
    }

    std::stable_sort(finite_indices.begin(), finite_indices.end(), [&](std::size_t lhs, std::size_t rhs) {
        return pl_orig.points[lhs].timestamp < pl_orig.points[rhs].timestamp;
    });

    const double first_point_time = pl_orig.points[finite_indices.front()].timestamp;
    const double last_point_time = pl_orig.points[finite_indices.back()].timestamp;
    const double scan_duration = std::max(1e-3, robosense_scan_duration_);
    const double timestamp_tolerance = std::max(0.0, robosense_timestamp_tolerance_);

    // Some M20 point-cloud publishers stamp the scan start, while others stamp
    // publication time after all points were acquired. Select a window in the
    // point timestamp domain and expose its real start time to LIO.
    const bool header_is_scan_start = first_point_time >= header_time - timestamp_tolerance &&
                                      first_point_time <= header_time + timestamp_tolerance;
    const double window_start =
        header_is_scan_start ? header_time - timestamp_tolerance : last_point_time - scan_duration;
    const double window_end =
        header_is_scan_start ? header_time + scan_duration : last_point_time + timestamp_tolerance;

    std::vector<std::size_t> valid_indices;
    valid_indices.reserve(finite_indices.size());
    for (const std::size_t index : finite_indices) {
        const double point_time = pl_orig.points[index].timestamp;
        if (point_time >= window_start && point_time <= window_end) {
            valid_indices.emplace_back(index);
        }
    }

    if (valid_indices.empty()) {
        LOG(WARNING) << "RoboSense scan has no points in selected time window";
        return;
    }

    const int timestamp_mode = header_is_scan_start ? 0 : 1;
    if (timestamp_mode != robosense_timestamp_mode_) {
        LOG(INFO) << "RoboSense timestamp mode: "
                  << (header_is_scan_start ? "header_is_scan_start" : "header_is_publish_time")
                  << ", header-to-point range: [" << std::setprecision(6)
                  << (first_point_time - header_time) * 1e3 << ", "
                  << (last_point_time - header_time) * 1e3 << "] ms";
        robosense_timestamp_mode_ = timestamp_mode;
    }

    last_scan_start_time_ =
        header_is_scan_start ? header_time : pl_orig.points[valid_indices.front()].timestamp;
    cloud_out_.header.stamp = static_cast<std::uint64_t>(last_scan_start_time_ * 1e6);

    const std::size_t discarded = pl_orig.points.size() - valid_indices.size();
    if (discarded > pl_orig.points.size() / 10) {
        LOG_EVERY_N(WARNING, 50) << "RoboSense fused scan clipped to one time window: kept "
                                 << valid_indices.size() << "/" << pl_orig.points.size()
                                 << " points, scan start: " << std::setprecision(14)
                                 << last_scan_start_time_ << ", header: " << header_time;
    }

    const int filter_num = std::max(1, point_filter_num_);
    std::unordered_map<std::uint16_t, int> ring_counts;
    ring_counts.reserve(256);

    for (const std::size_t index : valid_indices) {
        const auto &src = pl_orig.points[index];
        int &ring_count = ring_counts[src.ring];
        if ((ring_count++ % filter_num) != 0) {
            continue;
        }

        PointType added_pt;
        added_pt.x = src.x;
        added_pt.y = src.y;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;
        added_pt.timestamp = std::max(0.0, src.timestamp - last_scan_start_time_) * 1e3;

        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
            cloud_out_.points.push_back(added_pt);
        }
    }
    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace lightning
