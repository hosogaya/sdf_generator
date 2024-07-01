#include <sdf_generator_ros/tsdf_server.hpp>

namespace sdf_generator
{
TsdfServer::TsdfServer(const rclcpp::NodeOptions options)
: rclcpp::Node("tsdf_server", options)
{
    /**
     * Publisher and Subscriber
    */
    pub_layer_ = create_publisher<sdf_msgs::msg::Layer>(
        "output/layer", 1
    );
    pub_mesh_ = create_publisher<sdf_msgs::msg::Mesh>(
        "output/mesh", 1
    );
    sub_point_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/point_cloud", 1, std::bind(&TsdfServer::pointCloudCallback, this, std::placeholders::_1)
    );

    /**
     * Parameters
    */
    auto map_config = getTsdfMapConfig(this->get_node_logging_interface(), this->get_node_parameters_interface());
    auto integrator_config = getTsdfIntegratorConfig(this->get_node_logging_interface(), this->get_node_parameters_interface());
    auto mesh_integrator_config = getMeshIntegratorConfig(this->get_node_logging_interface(), this->get_node_parameters_interface());
    world_frame_ = declare_parameter("world_frame", "map");
    sensor_frame_ = declare_parameter("sensor_frame", "sensor_frame");
    std::string color_map_method = declare_parameter("color_map_method", "rainbow");
    std::string sensor_type = declare_parameter("sensor_type", "lidar");
    int mesh_publish_hz = declare_parameter<int>("mesh_publish_hz", 10);
    std::string mesh_color_mode_string = declare_parameter("mesh_color_mode", "color");

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

    RCLCPP_INFO(get_logger(), "Built tsdf map and integrator");
    
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

    RCLCPP_INFO(get_logger(), "Built color map");

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

    RCLCPP_INFO(get_logger(), "built point cloud processor");

    // mesh 
    mesh_color_mode_ = getColorMode(mesh_color_mode_string);
    mesh_layer_.reset(new MeshLayer(tsdf_map_->blockSize()));
    mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
        mesh_integrator_config, tsdf_map_->getTsdfLayerPtr().get(), mesh_layer_.get()
    ));

    RCLCPP_INFO(get_logger(), "built mesh integrator");

    // tf
    tf_buffer_.reset(new tf2_ros::Buffer(get_clock()));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    // timer
    auto mesh_dt = std::chrono::microseconds(int(1e6)/mesh_publish_hz);
    mesh_timer_ = create_wall_timer(mesh_dt, std::bind(&TsdfServer::meshTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Construction finished");
}

TsdfServer::~TsdfServer() {}

void TsdfServer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    RCLCPP_INFO(get_logger(), "point cloud callback is called");
    rclcpp::Time sub_time(msg->header.stamp);

    TransformMatrix<Scalar> tf_global2current;
    if (!getTransform(world_frame_, sensor_frame_, sub_time, tf_global2current)) return;

    RCLCPP_INFO(get_logger(), "get transform from %s to %s", world_frame_.c_str(), sensor_frame_.c_str());

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

    RCLCPP_INFO(get_logger(), "extracted point and colors");

    point_cloud_processor_->process(points_c, colors, normals_c);

    RCLCPP_INFO(get_logger(), "extracted normals");

    // icp

    // integrating
    tsdf_integrator_->integratePointArray(tf_global2current, points_c, normals_c, colors, false);
    RCLCPP_INFO(get_logger(), "integrated point cloud to tsdf map");

    // publish
    sdf_msgs::msg::Layer::UniquePtr layer_msg = toMsg(tsdf_map_->getTsdfLayerPtr());
    RCLCPP_INFO(get_logger(), "created layer msg");
    RCLCPP_INFO(get_logger(), "block num: %ld", layer_msg->blocks.size());
    pub_layer_->publish(std::move(layer_msg));
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
        return false;
    }

    return true;
}

void TsdfServer::meshTimerCallback()
{
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;

    // integrate mesh
    mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);

    // generate mesh msg
    sdf_msgs::msg::Mesh::UniquePtr mesh_msg(new sdf_msgs::msg::Mesh);
    generateMeshMsg(mesh_layer_.get(), mesh_color_mode_, mesh_msg.get());    
    mesh_msg->header.frame_id = world_frame_;
    mesh_msg->header.stamp = now();

    // pulish
    pub_mesh_->publish(std::move(mesh_msg));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sdf_generator::TsdfServer);