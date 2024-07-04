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

    auto esdf_config = getEsdfMapConfig(get_node_logging_interface(), get_node_parameters_interface());
    auto esdf_integrator_config = getEsdfIntegratorConfig(get_node_logging_interface(), get_node_parameters_interface());

    esdf_map_.reset(new EsdfMap(esdf_config));
    esdf_integrator_.reset(new EsdfIntegrator(
        esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(), 
        esdf_map_->getEsdfLayerPtr()
    ));

    publish_esdf_layer_ = declare_parameter<bool>("publish_esdf_layer", true);
    double esdf_publish_hz = declare_parameter<double>("esdf_publish_hz", 10.0);
    auto esdf_dt = std::chrono::microseconds(int(1e6/esdf_publish_hz));
    esdf_timer_ = create_wall_timer(esdf_dt, std::bind(&EsdfServer::esdfTimerCallback, this));
}

EsdfServer::~EsdfServer() {}

void EsdfServer::esdfTimerCallback()
{
    if (tsdf_map_->getTsdfLayerConstPtr()->blockNum() > 0)
    {
        bool clear_updated_flag_esdf = true;
        esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
        sdf_msgs::msg::EsdfLayer::UniquePtr msg = toMsg(esdf_map_->getEsdfLayerPtr(), tsdf_map_->getTsdfLayerPtr());
        pub_esdf_layer_->publish(std::move(msg));
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sdf_generator::EsdfServer);