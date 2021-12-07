#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_lidar_calibration_node");

    ros::NodeHandle nh();
    ros::NodeHandle nh_private("~");
}