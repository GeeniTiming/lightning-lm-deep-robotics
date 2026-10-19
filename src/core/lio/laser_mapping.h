#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"
#include "pointcloud_preprocess.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

namespace ui {
class PangolinWindow;
}

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    enum class RunStatus {
        NO_READY_SCAN,  // no scan, or the oldest scan is still waiting for IMU
        FRAME_CONSUMED, // a scan was consumed but intentionally produced no output
        OUTPUT_READY,   // a scan was consumed and produced a corrected state
    };

    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        /// 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    /// init without ros
    bool Init(const std::string &config_yaml);

    /// Process one queued LiDAR frame. Run() is kept for existing callers.
    RunStatus RunOnce();
    bool Run();

    /// Used by event-driven callers to drain all scans covered by buffered IMU.
    bool ShouldProcessLidar();
    bool HasPendingLidar();

    /// Finalize complete RoboSense time bins before an end-of-input map save.
    void FlushPendingPointClouds();

    // callbacks of lidar and imu
    /// 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    /// 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    /// 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    void SetInitPose(const SE3 &pose);

    /// 保存前端的地图
    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    /// 获取关键帧
    Keyframe::Ptr GetKeyframe() const { return last_kf_; }

    /// 获取激光的状态
    NavState GetState() const { return state_point_; }

    /// 获取经过回环优化后的位姿
    SE3 GetOptPose() const;

    /// 获取IMU状态
    NavState GetIMUState() const {
        std::lock_guard<std::mutex> lock(mtx_imu_state_);
        if (imu_initialized_.load(std::memory_order_acquire)) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    CloudPtr GetScanUndist() const { return scan_undistort_; }
    CloudPtr GetScanDownWorld() const { return scan_down_world_; }
    pcl::VoxelGrid<PointType>& GetVoxelFilter() { return voxel_scan_; }

    /// 获取最新的点云
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() { return all_keyframes_; }

    /**
     * 计算全局地图
     * @param use_lio_pose
     * @return
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

   private:
    // sync lidar with imu
    bool SyncPackages();

    void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    void OriObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot_ * (state_point_.offset_R_lidar_ * pi.getVector3fMap().cast<double>() +
                                            state_point_.offset_t_lidar_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    bool LoadParamsFromYAML(const std::string &yaml);

    /// 创建关键帧
    void MakeKF();

    /// Buffer helpers. Callers must hold mtx_buffer_.
    void EnqueueLidarCloud(CloudPtr cloud, double timestamp);
    void ResolveRobosenseInputMode(bool force);
    void BufferRobosenseCloud(CloudPtr cloud, double source_start_time);
    void FinalizeRobosenseBins(bool force);
    void TrimLidarBuffer();
    void TrimImuBuffer();

   private:
    Options options_;

    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    std::string map_file_path_;

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    std::vector<Vec4f> corr_pts_;                     // inlier pts
    std::vector<Vec4f> corr_norm_;                    // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    std::vector<float> residuals_;           // point-to-plane residuals
    std::vector<char> point_selected_surf_;  // selected points
    std::vector<Vec4f> plane_coef_;          // plane coeffs

    mutable std::mutex mtx_buffer_;
    mutable std::mutex mtx_imu_state_;
    std::mutex mtx_orientation_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;
    lightning::IMUPtr orientation_imu_ = nullptr;

    /// options
    bool keep_first_imu_estimation_ = false;    // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    enum class RobosenseInputMode { UNKNOWN, DIRECT_SCAN, REBATCH_POINTS };
    struct RobosenseProbeFrame {
        double header_time = 0.0;
        double point_start_time = 0.0;
        CloudPtr cloud;
    };
    RobosenseInputMode robosense_input_mode_ = RobosenseInputMode::UNKNOWN;
    bool robosense_rebatch_enabled_ = true;
    double robosense_scan_period_ = 0.1;
    double robosense_rebatch_delay_ = 0.3;
    double robosense_publish_time_threshold_ = 0.05;
    size_t robosense_min_bin_points_ = 100;
    size_t robosense_mode_probe_frames_ = 5;
    std::deque<RobosenseProbeFrame> robosense_probe_frames_;
    bool robosense_bin_origin_set_ = false;
    double robosense_bin_origin_ = 0.0;
    double robosense_latest_point_time_ = -1.0;
    std::int64_t robosense_last_emitted_bin_ = std::numeric_limits<std::int64_t>::min();
    std::map<std::int64_t, CloudPtr> robosense_bins_;
    size_t robosense_emitted_bins_ = 0;
    size_t robosense_dropped_bins_ = 0;
    size_t robosense_late_points_ = 0;

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;
    size_t max_lidar_buffer_size_ = 20;
    size_t max_imu_buffer_size_ = 4000;
    double imu_buffer_duration_ = 5.0;
    double imu_buffer_guard_time_ = 0.2;
    std::atomic_bool imu_initialized_ = false;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_num_ = 0, frame_num_ = 0;

    double last_lidar_time_ = 0;

    bool use_imu_orient_ = false;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    Vec3d pos_lidar_;  // lidar position after eskf update
    SO3 euler_cur_;    // rotation in euler angles
    bool extrinsic_est_en_ = true;
    bool use_aa_ = false;  // use anderson acceleration?

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H
