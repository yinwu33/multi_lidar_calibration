#ifndef MULTI_LIDAR_CALIBRATION_REGISTRATION_H
#define MULTI_LIDAR_CALIBRATION_REGISTRATION_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace multi_lidar_calibration
{
class Registration {
public:
    typedef pcl::PointXYZ POINT;
    typedef pcl::PointCloud<POINT> CLOUD;
    typedef pcl::PointCloud<POINT>::Ptr CLOUD_Ptr;

    Registration() = default;
    virtual void InitParameters() = 0;

    virtual void Match(CLOUD_Ptr input_source, CLOUD_Ptr input_target, Eigen::Matrix4f guess, CLOUD_Ptr output_cloud) = 0;

    virtual bool IsConverged() { return false; }

    virtual double Score() { return -1.0; }

    virtual Eigen::Matrix4f ResultPose() = 0;

protected:
    int test;
    YAML::Node config_node_;
};

} // namespace multi_lidar_calibration

#endif // MULTI_LIDAR_CALIBRATION_REGISTRATION_H