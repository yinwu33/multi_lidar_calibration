#ifndef MULTI_LIDAR_CALIBRATION_REGISTRATION_H
#define MULTI_LIDAR_CALIBRATION_REGISTRATION_H

#include<yaml-cpp/yaml.h>

namespace multi_lidar_calibration
{
class Registration {
public:

private:
    std::shared_ptr<YAML::Node>  config_node_ = nullptr;
};

} // namespace multi_lidar_calibration

#endif // MULTI_LIDAR_CALIBRATION_REGISTRATION_H