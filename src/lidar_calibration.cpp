#include "multi_lidar_calibration/lidar_calibration.h"

#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>


namespace multi_lidar_calibration
{
LidarCalibration::LidarCalibration(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private) {
    std::string method = nh_private_.param<std::string>("method", "ICP");
    std::string config_file = nh_private_.param<std::string>("config_file", "");

    ROS_INFO("====================STARTING CALIBRATION====================");
    ROS_INFO("Using method: %s", method.c_str());
    ROS_INFO("Config file path: %s", config_file.c_str());

    if (config_file == "") {
        ROS_FATAL("Please Check Config File Path!");
    }

    if (method == "icp" or method == "ICP") {
        method_ = Method::ICP;
        registrator_ = std::make_shared<ICPRegistration>(config_file);
    }
    else if (method == "ndt" or method == "NDT") {
        method_ = Method::NDT;
        // todo
    }
    else {
        ROS_FATAL("Method {%s} is not supported. Stop the process", method.c_str());
    }
    InitSubscribers();

    output_pub_ = std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::PointCloud2>("output_cloud", 10));

    pose_ = Eigen::Matrix4f::Identity();

}

void LidarCalibration::Run() {
    ROS_INFO("Waiting for parent topic: \"%s\" and child topic: \"%s\"", parent_cloud_name_.c_str(), child_cloud_name_.c_str());

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (DoOptimization()) {
            Logging();
        }

        rate.sleep();
    }
}

void LidarCalibration::InitSubscribers() {
    parent_cloud_name_ = nh_private_.param<std::string>("parent_cloud_name", "");
    child_cloud_name_ = nh_private_.param<std::string>("child_cloud_name", "");
    voxel_size_ = nh_private_.param<float>("voxel_size", 0.1);

    cloud_parent_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, parent_cloud_name_, 10);
    cloud_child_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, child_cloud_name_, 10);

    cloud_sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *cloud_parent_sub_, *cloud_child_sub);
    cloud_sync_->registerCallback(boost::bind(&LidarCalibration::Callback, this, _1, _2));
}

bool LidarCalibration::DoOptimization() {
    if (parent_cloud_ == nullptr or child_cloud_ == nullptr)
        return false;

    CLOUD_Ptr output_cloud(new CLOUD);


    registrator_->Match(child_filtered_cloud_, parent_cloud_, pose_, output_cloud);

    pose_ = registrator_->ResultPose();

    PublishCloud(output_cloud);

    TFBroadCaster();

    return true;
}

void LidarCalibration::Callback(const sensor_msgs::PointCloud2::ConstPtr& parent_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr& child_cloud_msg) {
    CLOUD_Ptr parent_cloud_temp(new CLOUD);
    CLOUD_Ptr child_cloud_temp(new CLOUD);
    CLOUD_Ptr child_cloud_filtered_temp(new CLOUD);

    current_time_ = parent_cloud_msg->header.stamp;

    pcl::fromROSMsg(*parent_cloud_msg, *parent_cloud_temp);
    pcl::fromROSMsg(*child_cloud_msg, *child_cloud_temp);

    parent_frame_id_ = parent_cloud_msg->header.frame_id;
    child_frame_id_ = child_cloud_msg->header.frame_id;

    if (voxel_size_ > 0) {
        VoxelFiter(child_cloud_temp, child_cloud_filtered_temp);
        child_filtered_cloud_ = child_cloud_filtered_temp;
    }
    else {
        child_filtered_cloud_ = child_cloud_temp;
    }

    parent_cloud_ = parent_cloud_temp;
    child_cloud_ = child_cloud_temp;
}

void LidarCalibration::VoxelFiter(CLOUD_Ptr input, CLOUD_Ptr output) {
    pcl::VoxelGrid<POINT> voxel_grid;

    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.filter(*output);
}

void LidarCalibration::Logging() {
    bool isConverged = registrator_->IsConverged();

    Eigen::Matrix3f rotation_matrix = pose_.block(0, 0, 3, 3);
    Eigen::Vector3f translation_vector = pose_.block(0, 3, 3, 1);

    if (isConverged)
        std::cout << "\n\n--------------------Converge--------------------" << std::endl;
    else
        std::cout << "\n\n----------------Wait for Converge---------------" << std::endl;

    std::cout << "Score: " << registrator_->Score()
        << "\n From " << child_frame_id_ << " To " << parent_frame_id_
        << "Pose: \n" << pose_ << std::endl;

    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
        << " " << rotation_matrix.eulerAngles(2, 1, 0).transpose() << " /" << parent_frame_id_
        << " /" << child_frame_id_ << " 10" << std::endl;
}

void LidarCalibration::PublishCloud(CLOUD_Ptr cloud) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = parent_frame_id_;

    output_pub_->publish(cloud_msg);
}

void LidarCalibration::TFBroadCaster() {
    Eigen::Affine3d aff;
    aff.matrix() = pose_.cast<double>();
    geometry_msgs::TransformStamped t_transform_static = tf2::eigenToTransform(aff);
    t_transform_static.header.frame_id = parent_frame_id_;
    t_transform_static.child_frame_id = child_frame_id_;
    t_transform_static.header.stamp = current_time_;
    tf_static_bc_.sendTransform(t_transform_static);
}
} // namespace multi_lidar_calibration
