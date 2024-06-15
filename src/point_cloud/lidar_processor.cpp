#include <sdf_generator/point_cloud/lidar_processor.hpp>

namespace sdf_generator
{
LidarProcessor::LidarProcessor(const CommonConfig& common_config, 
                            const LidarConfig& lidar_config)
: PointCloudProcessor(common_config), lidar_config_(lidar_config)
{}

LidarProcessor::~LidarProcessor() {}

Scalar LidarProcessor::projectPointToImage(const Point& point, int& u, int& v)
{
    Scalar depth = point.norm();
    Scalar yaw = std::atan2(point.y(), point.x());
    Scalar pitch = std::asin(point.z() / depth);

    // 0<= proj_x, proj_y < width or height
    Scalar proj_x = common_config_.width_*(yaw - lidar_config_.min_pitch_fov_rad_)/lidar_config_.yaw_fov_rad_range_;
    Scalar proj_y = common_config_.height_*(pitch - lidar_config_.min_pitch_fov_rad_)/lidar_config_.pitch_fov_rad_range_;

    u = std::round(proj_x);
    v = std::round(proj_y);
    
    if (u == common_config_.width_)
    {
        if (common_config_.is_loop_) u = 0;
        else return -1.0f; // do not use.
    }

    if (v > common_config_.height_ - 1 || v < 0)
        return -1.0f; // do not use.

    return depth;    
}
}