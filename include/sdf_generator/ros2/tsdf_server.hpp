#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sdf_generator/ros2/ros_params.hpp>
#include <sdf_generator/core/tsdf_map.hpp>
#include <sdf_generator/integrator/simple_tsdf_integrator.hpp>
#include <sdf_generator/point_cloud/point_cloud_processor.hpp>
#include <sdf_generator/point_cloud/conversions.hpp>

namespace sdf_generator
{
class TsdfServer : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfServer(const rclcpp::NodeOptions options);
    TsdfServer(const rclcpp::NodeOptions options, 
            const TsdfMap::Config& map_config,
            const TsdfIntegratorBase::Config& integrator_config);

    ~TsdfServer();

protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

    bool getTransform(const std::string& target, const std::string& source, 
                    const rclcpp::Time& sub_time, TransformMatrix<Scalar>& tf_mat);

    std::string world_frame_;
    std::string sensor_frame_;

    std::shared_ptr<TsdfMap> tsdf_map_;
    std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

    std::unique_ptr<PointCloudProcessor> point_cloud_processor_;
    ColorMap::Ptr color_map_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}