#ifndef MULTI_LIDAR_CALIBRATION_LIDAR_CALIBRATION_H
#define MULTI_LIDAR_CALIBRATION_LIDAR_CALIBRATION_H

#include <string>

#include <ros/ros.h>

#include "multi_lidar_calibration/models/registration.h"
#include "multi_lidar_calibration/models/icp_registration.h"


namespace multi_lidar_calibration
{
class LidarCalibration {
public:
    enum Method {
        ICP = 0,
        NDT = 1
    };
    LidarCalibration(ros::NodeHandle& nh, std::string method);

    void run();

private:
    ros::NodeHandle nh_;
    Method method_;

    std::string lidar_parent_topic_;
    std::string lidar_child_topic_;

    std::shared_ptr<Registration> registrator_;

};

} // namespace multi_lidar_calibration



#endif // MULTI_LIDAR_CALIBRATION_LIDAR_CALIBRATION_H