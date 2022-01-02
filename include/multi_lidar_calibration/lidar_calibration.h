#ifndef MULTI_LIDAR_CALIBRATION_LIDAR_CALIBRATION_H
#define MULTI_LIDAR_CALIBRATION_LIDAR_CALIBRATION_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "multi_lidar_calibration/models/registration.h"
#include "multi_lidar_calibration/models/icp_registration.h"


namespace multi_lidar_calibration
{
class LidarCalibration {
public:
    typedef pcl::PointXYZ POINT;
    typedef pcl::PointCloud<POINT> CLOUD;
    typedef pcl::PointCloud<POINT>::Ptr CLOUD_Ptr;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
public:
    enum Method {
        ICP = 0,
        NDT = 1
    };
    LidarCalibration(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    void Run();

    void InitSubscribers();

    bool DoOptimization();

    void Callback(const sensor_msgs::PointCloud2::ConstPtr &parent_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr &child_cloud_msg);

    void VoxelFiter(CLOUD_Ptr input, CLOUD_Ptr output);

    void Logging();

    void PublishCloud(CLOUD_Ptr cloud);

    void TFBroadCaster();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Method method_;

    ros::Time current_time_;

    std::string parent_cloud_name_;
    std::string child_cloud_name_;

    std::string parent_frame_id_;
    std::string child_frame_id_;

    std::shared_ptr<Registration> registrator_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_parent_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_child_sub;
    message_filters::Synchronizer<SyncPolicy> *cloud_sync_;

    std::shared_ptr<ros::Publisher> output_pub_;

    CLOUD_Ptr parent_cloud_ = nullptr;
    CLOUD_Ptr child_cloud_ = nullptr;
    CLOUD_Ptr child_filtered_cloud_ = nullptr;

    float voxel_size_;

    Eigen::Matrix4f pose_;

    tf2_ros::StaticTransformBroadcaster tf_static_bc_;

};

} // namespace multi_lidar_calibration



#endif // MULTI_LIDAR_CALIBRATION_LIDAR_CALIBRATION_H