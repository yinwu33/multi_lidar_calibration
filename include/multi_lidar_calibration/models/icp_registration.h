#include "multi_lidar_calibration/models/registration.h"

#include <string>

#include <pcl/registration/icp.h>

namespace multi_lidar_calibration
{
class ICPRegistration : public Registration {
public:
    ICPRegistration(const std::string& config_file);

    virtual void InitParameters() override;

    virtual void Match(CLOUD_Ptr input_source, CLOUD_Ptr input_target, Eigen::Matrix4f guess, CLOUD_Ptr output_cloud) override;

    virtual bool IsConverged() override;

    inline virtual double Score() override { return icp_ptr_->getFitnessScore(); }

    virtual Eigen::Matrix4f ResultPose() override { return icp_ptr_->getFinalTransformation(); }

private:
    pcl::IterativeClosestPoint<POINT, POINT>::Ptr icp_ptr_;
};

} // namespace multi_lidar_calibration
