#include <ros/ros.h>

#include "multi_lidar_calibration/lidar_calibration.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_lidar_calibration_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    multi_lidar_calibration::LidarCalibration calib(nh, nh_private);
    
    calib.Run();
}