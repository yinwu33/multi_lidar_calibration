#include "multi_lidar_calibration/lidar_calibration.h"

namespace multi_lidar_calibration
{

ICPRegistration::ICPRegistration(const std::string& config_file) : icp_ptr_(new pcl::IterativeClosestPoint<POINT, POINT>()){

  config_node_ = YAML::LoadFile(config_file);

  InitParameters();
}

void ICPRegistration::InitParameters() {
  icp_ptr_->setTransformationEpsilon(config_node_["epsilon"].as<double>());
  icp_ptr_->setMaximumIterations(config_node_["iteration_nums"].as<int>());
}

void ICPRegistration::Match(CLOUD_Ptr input_source, CLOUD_Ptr input_target, Eigen::Matrix4f guess, CLOUD_Ptr output_cloud) {
  icp_ptr_->setInputSource(input_source);
  icp_ptr_->setInputTarget(input_target);

  icp_ptr_->align(*output_cloud, guess);
}

bool ICPRegistration::IsConverged() {
  return icp_ptr_->hasConverged();
}

} // namespace multi_lidar_calibration
