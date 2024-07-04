#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/integrator/tsdf_integrator.hpp>
#include <sdf_generator/core/tsdf_map.hpp>
#include <sdf_generator/point_cloud/point_cloud_processor.hpp>
#include <sdf_generator/point_cloud/lidar_processor.hpp>
#include <sdf_generator/mesh/mesh_integrator.hpp>
#include <sdf_generator/core/esdf_map.hpp>
#include <sdf_generator/integrator/esdf_integrator.hpp>

namespace sdf_generator
{
inline bool getBoolParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    bool& value
)
{
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        desc.read_only = false;
        desc.dynamic_typing = true;

        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_BOOL != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }

        value = param.get_parameter_value().get<bool>();
    }
    else 
    {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_BOOL != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<bool>();
    }

    return true;
}

inline bool getStringParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    std::string& value
)
{
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        desc.read_only = false;
        desc.dynamic_typing = true;

        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_STRING != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }

        value = param.get_parameter_value().get<std::string>();
    }
    else {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_STRING != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<std::string>();
    }

    return true;
}

inline bool getIntParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    int& value
)
{
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        desc.read_only = false;
        desc.dynamic_typing = true;

        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_INTEGER != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }

        value = param.get_parameter_value().get<int>();
    }
    else {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_INTEGER != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<int>();
    }
    return true;
}

inline bool getDoubleParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    double& value
)
{
    RCLCPP_INFO(node_logger->get_logger(), "Read %s", param_name.c_str());
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        desc.read_only = false;
        desc.dynamic_typing = true;
        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_DOUBLE != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<double>();
    }
    else {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_DOUBLE != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<double>();
    }
    return true;
}

inline TsdfMap::Config getTsdfMapConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params) 
{
    RCLCPP_INFO(node_logger->get_logger(), "reading tsdf map config...");
    TsdfMap::Config tsdf_config;
    
    double voxel_size = tsdf_config.tsdf_voxel_size_;
    int voxels_per_side = tsdf_config.tsdf_voxels_per_side_;
    rcl_interfaces::msg::ParameterDescriptor desc_voxel_size;
    if (getDoubleParam("tsdf_voxel_size", node_logger, node_params, voxel_size)) 
        tsdf_config.tsdf_voxel_size_ = voxel_size;
    if (getIntParam("tsdf_voxels_per_side", node_logger, node_params, voxels_per_side)) 
        tsdf_config.tsdf_voxels_per_side_ = voxels_per_side;

    return tsdf_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    RCLCPP_INFO(node_logger->get_logger(), "reading tsdf integrator config..");
    TsdfIntegratorBase::Config integrator_config;

    integrator_config.voxel_carving_enabled_ = true;

    const TsdfMap::Config tsdf_config = getTsdfMapConfig(node_logger, node_params);

    double max_weight = integrator_config.max_weight_;
    double truncation_distance = -2.0;
    bool voxel_carving_enabled = integrator_config.voxel_carving_enabled_;
    double max_ray_length = integrator_config.max_ray_length_;
    double min_ray_length = integrator_config.min_ray_length_;

    if (getDoubleParam("truncation_distance", node_logger, node_params, truncation_distance))
        integrator_config.default_truncation_distance_ =
            truncation_distance > 0
                ? truncation_distance
                : -truncation_distance*tsdf_config.tsdf_voxel_size_;

    if (getBoolParam("voxel_carving_enabled", node_logger, node_params, voxel_carving_enabled))
        integrator_config.voxel_carving_enabled_ = voxel_carving_enabled;

    if (getDoubleParam("max_ray_length_m", node_logger, node_params, max_ray_length))
        integrator_config.max_ray_length_ = max_ray_length; 

    if (getDoubleParam("min_ray_length_m", node_logger, node_params, min_ray_length))
        integrator_config.min_ray_length_ = min_ray_length; 
    
    if (getDoubleParam("max_weight", node_logger, node_params, max_weight))
        integrator_config.max_weight_ = max_weight;

    bool use_const_weight = integrator_config.use_const_weight_;  
    if (getBoolParam("use_const_weight", node_logger, node_params, use_const_weight))
        integrator_config.use_const_weight_ = use_const_weight;

    bool use_weight_dropoff = integrator_config.use_weight_dropoff_;
    if (getBoolParam("use_weight_dropoff", node_logger, node_params, use_weight_dropoff))
        integrator_config.use_weight_dropoff_ = use_weight_dropoff;

    bool allow_clear = integrator_config.allow_clear_;
    if (getBoolParam("allow_clear", node_logger, node_params, allow_clear))
        integrator_config.allow_clear_ = allow_clear;

    double start_voxel_subsampling_factor = integrator_config.start_voxel_subsampling_factor_;
    if (getDoubleParam("start_voxel_subsampling_factor", node_logger, node_params, start_voxel_subsampling_factor))
        integrator_config.start_voxel_subsampling_factor_ = start_voxel_subsampling_factor;
    
    int max_consecutive_ray_collisions = integrator_config.max_consecutive_ray_collisions_;
    if (getIntParam("max_consecutive_ray_collisions", node_logger, node_params, max_consecutive_ray_collisions))
        integrator_config.max_consecutive_ray_collisions_ = max_consecutive_ray_collisions;

    int clear_checks_every_n_frames = integrator_config.clear_checks_every_n_frames_;
    if (getIntParam("clear_checks_every_n_frames", node_logger, node_params, clear_checks_every_n_frames))
        integrator_config.clear_checks_every_n_frames_ = clear_checks_every_n_frames;

    double max_integration_time_s = integrator_config.max_integration_time_s_;
    if (getDoubleParam("max_integration_time_s", node_logger, node_params, max_integration_time_s))
        integrator_config.max_integration_time_s_ = max_integration_time_s;

    bool anti_grazing = integrator_config.enable_anti_grazing_;
    if (getBoolParam("anti_grazing", node_logger,  node_params, anti_grazing))
        integrator_config.enable_anti_grazing_ = anti_grazing;

    bool use_sparsity_compensation_factor = integrator_config.use_sparsity_compensation_factor_;
    if (getBoolParam("use_sparsity_compensation_factor", node_logger, node_params, use_sparsity_compensation_factor))
        integrator_config.use_sparsity_compensation_factor_ = use_sparsity_compensation_factor;
    
    double sparsity_compensation_factor = integrator_config.sparsity_compensation_factor_;
    if (getDoubleParam("sparsity_compensation_factor", node_logger, node_params, sparsity_compensation_factor))
        integrator_config.sparsity_compensation_factor_ = sparsity_compensation_factor;
    
    std::string integration_order_mode = integrator_config.integration_order_mode_;
    if (getStringParam("integration_order_mode", node_logger, node_params, integration_order_mode))
        integrator_config.integration_order_mode_ = integration_order_mode;

    int integrator_threads = std::thread::hardware_concurrency();
    if (getIntParam("integrator_threads", node_logger, node_params, integrator_threads))
    {
        if (integrator_threads < 0) RCLCPP_ERROR(node_logger->get_logger(), "integrator threads must have positive value");
        else integrator_config.integrator_threads_ = integrator_threads;
    }
    
    bool merge_with_clear = integrator_config.merge_with_clear_;
    if (getBoolParam("merge_with_clear", node_logger, node_params, merge_with_clear))
        integrator_config.merge_with_clear_ = merge_with_clear;

    double weight_reduction_exp = integrator_config.weight_reduction_exp_;
    if (getDoubleParam("weight_reduction_exp", node_logger, node_params, weight_reduction_exp))
        integrator_config.weight_reduction_exp_ = weight_reduction_exp;

    double weight_dropoff_epsilon = integrator_config.weight_dropoff_epsilon_;
    if (getDoubleParam("weight_dropoff_epsilon", node_logger, node_params, weight_dropoff_epsilon))
        integrator_config.weight_dropoff_epsilon_ = weight_dropoff_epsilon;

    // bool normal_available = integrator_config.normal_available_;
    // if (getBoolParam("normal_available", node_logger, node_params, normal_available))
    //     integrator_config.normal_available_ = normal_available;

    double reliable_band_ratio = integrator_config.reliable_band_ratio_;
    if (getDoubleParam("reliable_band_ratio", node_logger, node_params, reliable_band_ratio))
        integrator_config.reliable_band_ratio_ = reliable_band_ratio;

    // bool curve_assumption = integrator_config.curve_assumption_;
    // if (getBoolParam("curve_assumption", node_logger, node_params, curve_assumption))
    //     integrator_config.curve_assumption_ = curve_assumption;

    double reliable_normal_ratio_thre = integrator_config.reliable_normal_ratio_thre_;
    if (getDoubleParam("reliable_normal_ratio_thre", node_logger, node_params, reliable_normal_ratio_thre))
        integrator_config.reliable_normal_ratio_thre_ = reliable_normal_ratio_thre;

    int max_number_of_rays = integrator_config.max_nubmer_of_rays_;
    if (getIntParam("max_number_of_rays", node_logger, node_params, max_number_of_rays))
        if (max_number_of_rays > 0) 
            integrator_config.max_nubmer_of_rays_ = max_number_of_rays;

    return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    EsdfMap::Config config;

    double d_value = config.esdf_voxel_size_;
    if (getDoubleParam("tsdf_voxel_size", node_logger, node_params, d_value))
        config.esdf_voxel_size_ = d_value;
    
    int i_value = config.esdf_voxels_per_side;
    if (getIntParam("tsdf_voxels_per_side", node_logger, node_params, i_value))
        config.esdf_voxels_per_side = i_value;

    return config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    EsdfIntegrator::Config config;
    // buffers
    double d_value;
    int i_value;
    bool b_value;

    d_value = config.max_distance_;
    if (getDoubleParam("esdf_max_distance", node_logger, node_params, d_value))
        config.max_distance_ = d_value;

    d_value = config.default_distance_;
    if (getDoubleParam("esdf_default_distance", node_logger, node_params, d_value))
        config.default_distance_ = d_value;

    d_value = config.max_behind_surface_;
    if (getDoubleParam("esdf_max_behind_surface", node_logger, node_params, d_value))
        config.max_behind_surface_ = d_value;

    d_value = config.band_distance_;
    if (getDoubleParam("esdf_band_distance", node_logger, node_params, d_value))
        config.band_distance_ = d_value;

    d_value = config.occ_voxel_size_ratio_;
    if (getDoubleParam("esdf_occ_voxel_size_ratio", node_logger, node_params, d_value))
        config.occ_voxel_size_ratio_ = d_value;

    d_value = config.min_weight_;
    if (getDoubleParam("esdf_min_weight", node_logger, node_params, d_value))
        config.min_weight_ = d_value;

    i_value = config.num_buckets_;
    if (getIntParam("esdf_num_buckets", node_logger, node_params, i_value))
        config.num_buckets_ = i_value;

    i_value = config.num_neighbor_;
    if (getIntParam("esdf_num_neighbor", node_logger, node_params, i_value))
        config.num_neighbor_ = i_value;

    b_value = config.patch_on_;
    if (getBoolParam("esdf_patch_on", node_logger, node_params, b_value))
        config.patch_on_ = b_value;

    b_value = config.early_break_;
    if (getBoolParam("esdf_early_break", node_logger, node_params, b_value))
        config.early_break_ = b_value;

    b_value = config.finer_esdf_on_;
    if (getBoolParam("esdf_finer_esdf_on", node_logger, node_params, b_value))
        config.finer_esdf_on_ = b_value;

    b_value = config.fixed_band_esdf_on_;
    if (getBoolParam("esdf_fixed_band_esdf_on", node_logger, node_params, b_value))
        config.fixed_band_esdf_on_ = b_value;

    d_value = config.gradient_sign_;
    if (getDoubleParam("esdf_gradient_sign", node_logger, node_params, d_value))
        config.gradient_sign_ = d_value >=0.0 ? 1.0 : -1.0;
    
    b_value = config.allocate_tsdf_in_range_;
    if (getBoolParam("esdf_allocate_tsdf_in_range", node_logger, node_params, b_value))
        config.allocate_tsdf_in_range_ = b_value;
    
    i_value = config.range_boundary_offset_.x();
    if (getIntParam("esdf_range_boundary_offset_x", node_logger, node_params, i_value))
        config.range_boundary_offset_.x() = i_value;

    i_value = config.range_boundary_offset_.y();
    if (getIntParam("esdf_range_boundary_offset_y", node_logger, node_params, i_value))
        config.range_boundary_offset_.y() = i_value;

    i_value = config.range_boundary_offset_.z();
    if (getIntParam("esdf_range_boundary_offset_z", node_logger, node_params, i_value))
        config.range_boundary_offset_.z() = i_value;

    return config;
}

inline PointCloudProcessor::CommonConfig getPointCloudProcessorCommonConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    PointCloudProcessor::CommonConfig config;
    getIntParam("sensor_image_width", node_logger, node_params, config.width_);
    getIntParam("sensor_image_height", node_logger, node_params, config.height_);
    
    double min_d = config.min_d_;
    if (getDoubleParam("point_cloud_min_depth", node_logger, node_params, min_d))
        config.min_d_ = min_d;
    
    double min_z = config.min_z_;
    if (getDoubleParam("point_cloud_min_z", node_logger, node_params, min_z))
        config.min_z_ = min_z;

    double depth_smooth_thres_ratio = config.depth_smooth_thres_ratio_;
    if (getDoubleParam("depth_smooth_thres_ratio", node_logger, node_params, depth_smooth_thres_ratio))
        config.depth_smooth_thres_ratio_ = depth_smooth_thres_ratio;
    
    getBoolParam("point_cloud_is_loop", node_logger, node_params, config.is_loop_);

    return config;
}

inline LidarProcessor::LidarConfig getLidarConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    LidarProcessor::LidarConfig config;
    
    double min_yaw_fov_rad = config.min_yaw_fov_rad_;
    if (getDoubleParam("min_yaw_fov_rad", node_logger, node_params, min_yaw_fov_rad))
        config.min_yaw_fov_rad_ = min_yaw_fov_rad;

    double yaw_fov_rad_range = config.yaw_fov_rad_range_;
    if (getDoubleParam("yaw_fov_rad_range", node_logger, node_params, yaw_fov_rad_range))
        config.yaw_fov_rad_range_ = yaw_fov_rad_range;
    
    double min_pitch_fov_rad = config.min_pitch_fov_rad_;
    if (getDoubleParam("min_pitch_fov_rad", node_logger, node_params, min_pitch_fov_rad))
        config.min_pitch_fov_rad_ = min_pitch_fov_rad;

    double pitch_fov_rad_range = config.pitch_fov_rad_range_;
    if (getDoubleParam("pitch_fov_rad_range", node_logger, node_params, pitch_fov_rad_range))
        config.pitch_fov_rad_range_ = pitch_fov_rad_range;

    return config;
}

inline MeshIntegratorConfig getMeshIntegratorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    RCLCPP_INFO(node_logger->get_logger(), "reading mesh integrator config");
    MeshIntegratorConfig config;

    double min_weight = config.min_weight_;
    if (getDoubleParam("mesh_min_weight", node_logger, node_params, min_weight))
        config.min_weight_ = min_weight;
    
    getBoolParam("mesh_use_color", node_logger, node_params, config.use_color_);

    int threads_num = int(config.integrator_threads_);
    if (getIntParam("integrator_threads", node_logger, node_params, threads_num))
        config.integrator_threads_ = threads_num;

    return config;
}

}