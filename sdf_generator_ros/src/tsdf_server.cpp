#include <sdf_generator_ros/tsdf_server.hpp>

namespace sdf_generator
{
TsdfServer::TsdfServer(const rclcpp::NodeOptions options)
: TsdfServer(options, 
    getTsdfMapConfig(this->get_node_logging_interface(), this->get_node_parameters_interface()), 
    getTsdfIntegratorConfig(this->get_node_logging_interface(), this->get_node_parameters_interface()))
{}

TsdfServer::TsdfServer(const rclcpp::NodeOptions options, 
            const TsdfMap::Config& map_config,
            const TsdfIntegratorBase::Config& integrator_config)
: rclcpp::Node("tsdf_server", options)
{
    /**
     * Publisher and Subscriber
    */
    sub_point_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/point_cloud", 1, std::bind(&TsdfServer::pointCloudCallback, this, std::placeholders::_1)
    );

    /**
     * Parameters
    */
    world_frame_ = declare_parameter("world_frame", "map");
    sensor_frame_ = declare_parameter("sensor_frame", "sensor_frame");
    std::string color_map_method = declare_parameter("color_map_method", "rainbow");
    std::string sensor_type = declare_parameter("sensor_type", "lidar");

    /**
     * Construction
    */
    tsdf_map_ = std::make_shared<TsdfMap>(map_config);
    // tsdf integrator
    if (integrator_config.integration_order_mode_ == "simple") 
        tsdf_integrator_.reset(new SimpleTsdfIntegrator(integrator_config, tsdf_map_->getTsdfLayerPtr()));
    else 
    {
        RCLCPP_ERROR(get_logger(), "Please set specific integration method");
        rclcpp::shutdown();
    }
    
    // color map
    if (color_map_method == "rainbow")
        color_map_.reset(new RainbowColorMap());
    else if (color_map_method == "grayscale")
        color_map_.reset(new GrayScaleColorMap());
    else
    {
        RCLCPP_ERROR(get_logger(), "Please set specific color map method");
        rclcpp::shutdown();
    }

    // point cloud processor
    if (sensor_type == "lidar")
    {
        point_cloud_processor_.reset(new LidarProcessor(
            getPointCloudProcessorCommonConfig(get_node_logging_interface(), get_node_parameters_interface()),
            getLidarConfig(get_node_logging_interface(), get_node_parameters_interface())
        ));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Please set specific sensor type");
        rclcpp::shutdown();
    }

    // tf
    tf_buffer_.reset(new tf2_ros::Buffer(get_clock()));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

}

TsdfServer::~TsdfServer() {}

void TsdfServer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    rclcpp::Time sub_time(msg->header.stamp);

    TransformMatrix<Scalar> tf_global2current;
    if (!getTransform(world_frame_, sensor_frame_, sub_time, tf_global2current)) return;

    bool has_color(false);
    bool has_intensity(false);

    for (size_t d=0; d<msg->fields.size(); ++d)
    {
        if (msg->fields[d].name == "rgb")
        {
            msg->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            has_color = true;
        }
        else if (msg->fields[d].name == "intensity")
            has_intensity = true;
    }

    PointArray points_c;
    Vector3Array normals_c;
    ColorArray colors;
    
    if (has_color)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *point_cloud);
        convertPointCloud(*point_cloud, color_map_, points_c, colors);
    }
    else if (has_intensity)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *point_cloud);
        convertPointCloud(*point_cloud, color_map_, points_c, colors);
    }
    else 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *point_cloud);
        convertPointCloud(*point_cloud, color_map_, points_c, colors);
    }

    point_cloud_processor_->process(points_c, colors, normals_c);

    // icp

    // integrating
    tsdf_integrator_->integratePointArray(tf_global2current, points_c, normals_c, colors, false);

    // publish
}

bool TsdfServer::getTransform(const std::string& target, const std::string& source, 
                const rclcpp::Time& sub_time, TransformMatrix<Scalar>& tf_mat)
{
    try
    {
        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(target, source, sub_time);

        Eigen::Vector3<Scalar> translation(
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z
        );
        Eigen::Quaternion<Scalar> rotation(
            tf_msg.transform.rotation.w,
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z
        );

        tf_mat = TransformMatrix<Scalar>(translation, rotation);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10, "%s", ex.what());
    }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sdf_generator::TsdfServer);