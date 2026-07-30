//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <csignal>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

// Signal handler for SIGINT
void SignalHandler(int) { rclcpp::shutdown(); }

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning;

    /// Initialize ROS2
    rclcpp::init(argc, argv);

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    // Register the signal handler
    std::signal(SIGINT, SignalHandler);

    slam.StartSLAM("");
    slam.Spin();

    LOG(INFO) << "Saving map and path before shutdown...";
    slam.SaveMap();
    slam.SavePath();
    Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}
