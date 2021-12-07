#include "multi_lidar_calibration/models/registration.h"

#include <string>

#include <pcl/registration/icp.h>

namespace multi_lidar_calibration
{
class ICPRegistration : public Registration {
public:
    typedef pcl::PointXYZ POINT;
    typedef pcl::PointCloud<POINT> CLOUD;
    typedef pcl::PointCloud<POINT>::Ptr CLOUD_Ptr;

private:
    pcl::IterativeClosestPoint<POINT, POINT>::Ptr icp_ptr_;
};

} // namespace multi_lidar_calibration
