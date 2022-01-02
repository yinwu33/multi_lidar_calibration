#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/conditional_removal.h>

#include <yaml-cpp/yaml.h>


namespace cloud_cropper {

class CloudCropper {
public:
    CloudCropper(ros::NodeHandle &nh_, std::string input_topic, std::string output_topic, double time_offset_); // one box
    CloudCropper(ros::NodeHandle &nh_, std::string config_file); // multi boxes

    void msgCallbackSingle(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void msgCallbackMulti(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void parseYaml(YAML::Node &crop_config);

private:
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    pcl::ConditionalRemoval<pcl::PointXYZI> filter;

    std::vector<std::vector<double>> config_list;

    double time_offset = 0;
};

}