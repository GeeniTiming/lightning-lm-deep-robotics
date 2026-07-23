#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/measure_group.h"
#include "common/point_def.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lightning {

enum class LidarType { AVIA = 1, VELO32, OUST64, RoboSense };

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 *
 * 预处理程序
 * 主要是对各种不同的雷达处理时间戳差异
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    void Process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudType::Ptr &pcl_out);

    void Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud, PointCloudType::Ptr &pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    float &TimeScale() { return time_scale_; }
    double &RobosenseScanDuration() { return robosense_scan_duration_; }
    double &RobosenseTimestampTolerance() { return robosense_timestamp_tolerance_; }
    double LastScanStartTime() const { return last_scan_start_time_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }
    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }

   private:
    void Oust64Handler(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg);
    void RobosenseHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    double robosense_scan_duration_ = 0.12;
    double robosense_timestamp_tolerance_ = 0.005;
    double last_scan_start_time_ = 0.0;
    int robosense_timestamp_mode_ = -1;
    bool given_offset_time_ = false;

    float height_max_ = 1.0;
    float height_min_ = -1.0;
};
}  // namespace lightning

#endif
