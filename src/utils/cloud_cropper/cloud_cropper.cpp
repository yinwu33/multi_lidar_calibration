#include "multi_lidar_calibration/utils/cloud_cropper/cloud_cropper.h"

namespace cloud_cropper {

CloudCropper::CloudCropper(ros::NodeHandle& nh_, std::string input_topic, std::string output_topic, double time_offset_) : nh(nh_), time_offset(time_offset_) {
    cloud_sub = nh.subscribe(input_topic, 10, &CloudCropper::msgCallbackSingle, this);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
}

CloudCropper::CloudCropper(ros::NodeHandle& nh_, std::string config_file) : nh(nh_) {

    YAML::Node config = YAML::LoadFile(config_file);
    YAML::Node crop_config = config["boxes"];

    std::string input_topic = config["input_topic"].as<std::string>();
    std::string output_topic = config["output_topic"].as<std::string>();
    ROS_INFO("Input Toic: %s", input_topic.c_str());
    ROS_INFO("Output Topic: %s", output_topic.c_str());


    cloud_sub = nh.subscribe(input_topic, 10, &CloudCropper::msgCallbackMulti, this);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);

    parseYaml(crop_config);
}

void CloudCropper::parseYaml(YAML::Node& crop_config) {
    for (YAML::const_iterator it = crop_config.begin(); it != crop_config.end(); ++it) {
        // std::cout << it->second std::endl;
        std::vector<double> box;
        box.push_back(it->second["x"].as<double>());
        box.push_back(it->second["y"].as<double>());
        box.push_back(it->second["z"].as<double>());
        box.push_back(it->second["a"].as<double>());
        box.push_back(it->second["b"].as<double>());
        box.push_back(it->second["c"].as<double>());
        config_list.push_back(box);
    }
}


void CloudCropper::msgCallbackSingle(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // get parameters
    double x = nh.param<double>("x", 0);
    double y = nh.param<double>("y", 0);
    double z = nh.param<double>("z", 0);
    double a = nh.param<double>("a", 100);
    double b = nh.param<double>("b", 100);
    double c = nh.param<double>("c", 100);

    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    // filter cloud
    for (size_t i = 0; i < input_cloud.size(); ++i) {
        pcl::PointXYZI point = input_cloud.at(i);

        if ((point.x >= x and point.x <= x + a) and (point.y >= y and point.y <= y + b) and (point.z >= z and point.z <= z + c)) {
            output_cloud.push_back(point);
        }
    }

    // publish output pointcloud
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = msg->header;
    cloud_pub.publish(output_msg);
}

void CloudCropper::msgCallbackMulti(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    // filter cloud
    for (size_t i = 0; i < input_cloud.size(); ++i) {
        pcl::PointXYZI point = input_cloud.at(i);
        bool to_add = false;

        for (const auto box : config_list) {
            // std::cout << it->second std::endl;
            double x = box[0];
            double y = box[1];
            double z = box[2];
            double a = box[3];
            double b = box[4];
            double c = box[5];

            if (((point.x >= x and point.x <= x + a) and (point.y >= y and point.y <= y + b) and (point.z >= z and point.z <= z + c))) {
                to_add = true;
                break;
            }
        }
        if (to_add) {
            output_cloud.push_back(point);
        }
    }

    // publish output pointcloud
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = msg->header;
    ros::Time new_time = msg->header.stamp + ros::Duration(time_offset);
    output_msg.header.stamp = new_time;

    cloud_pub.publish(output_msg);
}


}