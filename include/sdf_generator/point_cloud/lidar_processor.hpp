#pragma once

#include <sdf_generator/point_cloud/point_cloud_processor.hpp>

namespace sdf_generator
{
class LidarProcessor : public PointCloudProcessor
{
public:
    struct LidarConfig
    {
        Scalar min_yaw_fov_rad_;
        Scalar yaw_fov_rad_range_;
        Scalar min_pitch_fov_rad_;
        Scalar pitch_fov_rad_range_;
    };

    LidarProcessor(const CommonConfig& common_config, const LidarConfig& lidar_config);
    ~LidarProcessor();

protected:
    Scalar projectPointToImage(const Point& point, int& u, int& v) override;

    const LidarConfig lidar_config_;
};
}