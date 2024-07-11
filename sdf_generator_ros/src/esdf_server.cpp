#include <sdf_generator_ros/esdf_server.hpp>

namespace sdf_generator
{
EsdfServer::EsdfServer(const rclcpp::NodeOptions options)
: TsdfServer(options)
{
    pub_esdf_layer_ = create_publisher<sdf_msgs::msg::EsdfLayer>
    (
        "output/esdf_layer", 1
    );

    publish_tsdf_layer_ = declare_parameter<bool>("pub_tsdf_layer", false);
    if (!publish_tsdf_layer_)
    {
        pub_tsdf_layer_.reset();
    }

    auto esdf_config = getEsdfMapConfig(get_node_logging_interface(), get_node_parameters_interface());
    auto esdf_integrator_config = getEsdfIntegratorConfig(get_node_logging_interface(), get_node_parameters_interface());

    esdf_map_.reset(new EsdfMap(esdf_config));
    esdf_integrator_.reset(new EsdfIntegrator(
        esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(), 
        esdf_map_->getEsdfLayerPtr()
    ));
}

EsdfServer::~EsdfServer() {}

void EsdfServer::layerTimerCallback()
{
    if (tsdf_map_->getTsdfLayerConstPtr()->blockNum() > 0)
    {
        RCLCPP_INFO(get_logger(), "[esdfTimerCallback] updating the esdf layer");
        bool clear_updated_flag_esdf = true;
        esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
        sdf_msgs::msg::EsdfLayer::UniquePtr msg = esdfLayer2Msg(esdf_map_->getEsdfLayerPtr());
        msg->header.frame_id = world_frame_;
        msg->header.stamp = last_point_cloud_time_;
        RCLCPP_INFO_STREAM(get_logger(), "[EsdfServer::layerTimerCallback] msg block size: " << msg->blocks.size());
        pub_esdf_layer_->publish(std::move(msg));
    }

    if (publish_tsdf_layer_)
    {
        sdf_msgs::msg::TsdfLayer::UniquePtr msg = tsdfLayer2Msg(tsdf_map_->getTsdfLayerPtr());
        msg->header.frame_id = world_frame_;
        msg->header.stamp = last_point_cloud_time_;
        pub_tsdf_layer_->publish(std::move(msg));
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sdf_generator::EsdfServer);