#pragma once

#include <sdf_generator/point_cloud/point_cloud_processor.hpp>

namespace sdf_generator
{
class LidarProcessor : public PointCloudProcessor
{
public:
    struct LidarConfig
    {
        Scalar min_yaw_fov_rad_ = -M_PI;
        Scalar yaw_fov_rad_range_ = 2*M_PI;
        Scalar min_pitch_fov_rad_ = -M_PI_2;
        Scalar pitch_fov_rad_range_ = M_PI;
    };

    LidarProcessor(const CommonConfig& common_config, const LidarConfig& lidar_config);
    ~LidarProcessor();

protected:
    Scalar projectPointToImage(const Point& point, int& u, int& v) override;

    const LidarConfig lidar_config_;
};
}