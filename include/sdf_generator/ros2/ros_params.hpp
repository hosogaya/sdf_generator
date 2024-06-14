#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/integrator/tsdf_integrator.hpp>
#include <sdf_generator/core/tsdf_map.hpp>

namespace sdf_generator
{
inline bool getBoolParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    bool& value
)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = param_name;
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.read_only = false;
    desc.dynamic_typing = true;

    node_params->declare_parameter(desc.name, rclcpp::ParameterValue(), desc);

    rclcpp::Parameter param;
    const bool got_param = node_params->get_parameter(desc.name, param);

    if (!got_param) return false;
    if (rclcpp::PARAMETER_BOOL != param.get_type())
    {
        RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
        return false;
    }

    value = param.get_parameter_value().get<bool>();
    return true;
}

inline bool getStringParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    std::string& value
)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = param_name;
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    desc.read_only = false;
    desc.dynamic_typing = true;

    node_params->declare_parameter(desc.name, rclcpp::ParameterValue(), desc);

    rclcpp::Parameter param;
    const bool got_param = node_params->get_parameter(desc.name, param);

    if (!got_param) return false;
    if (rclcpp::PARAMETER_STRING != param.get_type())
    {
        RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
        return false;
    }

    value = param.get_parameter_value().get<std::string>();
    return true;
}

inline bool getIntParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    int& value
)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = param_name;
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    desc.read_only = false;
    desc.dynamic_typing = true;

    node_params->declare_parameter(desc.name, rclcpp::ParameterValue(), desc);

    rclcpp::Parameter param;
    const bool got_param = node_params->get_parameter(desc.name, param);

    if (!got_param) return false;
    if (rclcpp::PARAMETER_INTEGER != param.get_type())
    {
        RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
        return false;
    }

    value = param.get_parameter_value().get<int>();

    return true;
}

inline bool getDoubleParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    double& value
)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = param_name;
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    desc.read_only = false;
    desc.dynamic_typing = true;

    node_params->declare_parameter(desc.name, rclcpp::ParameterValue(), desc);

    rclcpp::Parameter param;
    const bool got_param = node_params->get_parameter(desc.name, param);

    if (!got_param) return false;
    if (rclcpp::PARAMETER_DOUBLE != param.get_type())
    {
        RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
        return false;
    }

    value = param.get_parameter_value().get<double>();
    return true;
}

inline TsdfMap::Config getTsdfMapConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params) 
{
    TsdfMap::Config tsdf_config;
    
    double voxel_size = tsdf_config.tsdf_voxel_size_;
    int voxels_per_side = tsdf_config.tsdf_voxels_per_side_;
    rcl_interfaces::msg::ParameterDescriptor desc_voxel_size;
    if (getDoubleParam("tsdf_voxel_size", node_logger, node_params, voxel_size)) 
        tsdf_config.tsdf_voxel_size_ = voxel_size;
    if (getIntParam("tsdf_voxesl_per_side", node_logger, node_params, voxels_per_side)) 
        tsdf_config.tsdf_voxels_per_side_ = voxels_per_side;

    return tsdf_config;
}


// inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
//     const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
//     const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
// )
// {
//     TsdfIntegratorBase::Config integrator_config;

//     integrator_config.voxel_carving_enabled_ = true;

//     const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node_logger, node_params);

//     double max_weight = integrator_config.max_weight_;
//     double truncation_distance = -2.0;
//     bool voxel_carving_enabled_ = integrator_config.voxel_carving_enabled_;
//     double max_ray_length = integrator_config.max_ray_length_;
//     double min_ray_length = integrator_config.min_ray_length_;

//     if (getDoubleParam("truncation_distance", node_logger, node_params, truncation_distance))
//         integrator_config.default_truncation_distance_ =
//             truncation_distance > 0
//                 ? truncation_distance
//                 : -truncation_distance*tsdf_config.tsdf_voxel_size_;

//     if (getBoolParam("voxel_carving_enabled_", node_logger, node_params, voxel_carving_enabled_))
//         integrator_config.voxel_carving_enabled_ = voxel_carving_enabled_;

//     if (getDoubleParam("max_ray_length_m", node_logger, node_params, max_ray_length))
//         integrator_config.max_ray_length_ = max_ray_length; 

//     if (getDoubleParam("min_ray_length_m", node_logger, node_params, min_ray_length))
//         integrator_config.min_ray_length_ = min_ray_length; 
    
//     if (getDoubleParam("max_weight", node_logger, node_params, max_weight))
//         integrator_config.max_weight_ = max_weight;

//     bool use_const_weight = integrator_config.use_const_weight_;  
//     if (getBoolParam("use_const_weight", node_logger, node_params, use_const_weight))
//         integrator_config.use_const_weight_ = use_const_weight;

//     bool use_weight_dropoff_ = integrator_config.use_weight_dropoff_;
//     if (getBoolParam("use_weight_dropoff_", node_logger, node_params, use_weight_dropoff_))
//         integrator_config.use_weight_dropoff_ = use_weight_dropoff_;

//     bool allow_clear = integrator_config.allow_clear_;
//     if (getBoolParam("allow_clear", node_logger, node_params, allow_clear))
//         integrator_config.allow_clear_ = allow_clear;

//     double start_voxel_subsampling_factor = integrator_config.start_voxel_subsampling_factor_;
//     if (getDoubleParam("start_voxel_subsampling_factor", node_logger, node_params, start_voxel_subsampling_factor))
//         integrator_config.start_voxel_subsampling_factor_ = start_voxel_subsampling_factor;
    
//     int max_consecutive_ray_collisions = integrator_config.max_consecutive_ray_collisions_;
//     if (getIntParam("max_consecutive_ray_collisions", node_logger, node_params, max_consecutive_ray_collisions))
//         integrator_config.max_consecutive_ray_collisions_ = max_consecutive_ray_collisions;

//     int clear_checks_every_n_frames = integrator_config.clear_checks_every_n_frames_;
//     if (getIntParam("clear_checks_every_n_frames", node_logger, node_params, clear_checks_every_n_frames))
//         integrator_config.clear_checks_every_n_frames_ = clear_checks_every_n_frames;

//     double max_integration_time_s = integrator_config.max_integration_time_s_;
//     if (getDoubleParam("max_integration_time_s", node_logger, node_params, max_integration_time_s))
//         integrator_config.max_integration_time_s_ = max_integration_time_s;

//     bool anti_grazing = integrator_config.enable_anti_grazing_;
//     if (getBoolParam("anti_grazing", node_logger,  node_params, anti_grazing))
//         integrator_config.enable_anti_grazing_ = anti_grazing;

//     bool use_sparsity_compensation_factor = integrator_config.use_sparsity_compensation_factor_;
//     if (getBoolParam("use_sparsity_compensation_factor", node_logger, node_params, use_sparsity_compensation_factor))
//         integrator_config.use_sparsity_compensation_factor_ = use_sparsity_compensation_factor;
    
//     double sparsity_compensation_factor = integrator_config.sparsity_compensation_factor_;
//     if (getDoubleParam("sparsity_compensation_factor", node_logger, node_params, sparsity_compensation_factor))
//         integrator_config.sparsity_compensation_factor_ = sparsity_compensation_factor;
    
//     std::string integration_order_mode = integrator_config.integration_order_mode_;
//     if (getStringParam("integration_order_mode", node_logger, node_params, integration_order_mode))
//         integrator_config.integration_order_mode_ = integration_order_mode;

//     int integrator_threads = std::thread::hardware_concurrency();
//     if (getIntParam("integrator_threads", node_logger, node_params, integrator_threads))
//         if (integrator_threads < 0) RCLCPP_ERROR(node_logger->get_logger(), "integrator threads must have positive value");
//         else integrator_config.integrator_threads_ = integrator_threads;

//     bool merge_with_clear = integrator_config.merge_with_clear_;
//     if (getBoolParam("merge_with_clear", node_logger, node_params, merge_with_clear))
//         integrator_config.merge_with_clear_ = merge_with_clear;


//     return integrator_config;
// }

inline TsdfIntegratorBase::Config getTsdfIntegratorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    TsdfIntegratorBase::Config integrator_config;

    integrator_config.voxel_carving_enabled_ = true;

    const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node_logger, node_params);

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
        if (integrator_threads < 0) RCLCPP_ERROR(node_logger->get_logger(), "integrator threads must have positive value");
        else integrator_config.integrator_threads_ = integrator_threads;

    bool merge_with_clear = integrator_config.merge_with_clear_;
    if (getBoolParam("merge_with_clear", node_logger, node_params, merge_with_clear))
        integrator_config.merge_with_clear_ = merge_with_clear;

    double weight_reduction_exp = integrator_config.weight_reduction_exp_;
    if (getDoubleParam("weight_reduction_exp", node_logger, node_params, weight_reduction_exp))
        integrator_config.weight_reduction_exp_ = weight_reduction_exp;

    double weight_dropoff_epsilon = integrator_config.weight_dropoff_epsilon_;
    if (getDoubleParam("weight_dropoff_epsilon", node_logger, node_params, weight_dropoff_epsilon))
        integrator_config.weight_dropoff_epsilon_ = weight_dropoff_epsilon;

    bool normal_available = integrator_config.normal_available_;
    if (getBoolParam("normal_available", node_logger, node_params, normal_available))
        integrator_config.normal_available_ = normal_available;

    double reliable_band_ratio = integrator_config.reliable_band_ratio_;
    if (getDoubleParam("reliable_band_ratio", node_logger, node_params, reliable_band_ratio))
        integrator_config.reliable_band_ratio_ = reliable_band_ratio;

    bool curve_assumption = integrator_config.curve_assumption_;
    if (getBoolParam("curve_assumption", node_logger, node_params, curve_assumption))
        integrator_config.curve_assumption_ = curve_assumption;

    double reliable_normal_ratio_thre = integrator_config.reliable_normal_ratio_thre_;
    if (getDoubleParam("reliable_normal_ratio_thre", node_logger, node_params, reliable_normal_ratio_thre))
        integrator_config.reliable_normal_ratio_thre_ = reliable_normal_ratio_thre;

    return integrator_config;
}

}