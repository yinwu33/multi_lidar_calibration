#include <ros/ros.h>

#include <string>

#include "multi_lidar_calibration/utils/cloud_cropper/cloud_cropper.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_cropper_node");

    ros::NodeHandle nh;

    // get input pointcloud topic name
    std::string mode = nh.param<std::string>("mode", "");
    

    if (mode == "single") {
        ROS_INFO("Cropper Single Box Mode");

        std::string input_topic = nh.param<std::string>("input_topic", "/lidar_1/lidar_node/pointcloud");
        std::string output_topic = nh.param<std::string>("output_topic", "/lidar_1/pointcloud_filtered");
        double time_offset = nh.param<double>("time_offset", 0);

        ROS_INFO("Input Toic: %s", input_topic.c_str());
        ROS_INFO("Output Topic: %s", output_topic.c_str());

        cloud_cropper::CloudCropper cloud_cropper(nh, input_topic, output_topic, time_offset);
        ros::spin();
    }
    else {
        ROS_INFO("Cropper Multi Box Mode");

        std::string config_file = nh.param<std::string>("config", "");
        ROS_INFO("Config File: %s", config_file.c_str());
        
        cloud_cropper::CloudCropper cloud_cropper(nh, config_file);
        ros::spin();   
    }
}