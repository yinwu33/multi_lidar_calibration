#include "multi_lidar_calibration/lidar_calibration.h"

namespace multi_lidar_calibration
{

LidarCalibration::LidarCalibration(ros::NodeHandle& nh, std::string method) : nh_(nh) {
    if (method == "icp" or method == "ICP") {
        method_ = Method::ICP;
        registrator_ = std::make_shared<ICPRegistration>();
    }
    else if (method == "ndt" or method == "NDT") {
        method_ = Method::NDT;
        // todo
    }
    else {
        ROS_FATAL("Method {%s} is not supported. Stop the process", method.c_str());
    }

}

void LidarCalibration::run() {
    ROS_INFO("Waiting for parent topic: %s and child topic: %s", lidar_parent_topic_.c_str(), lidar_child_topic_.c_str());

    while (ros::ok()) {
        ros::spinOnce();
    }
}

} // namespace multi_lidar_calibration
