#include <sdf_generator/ros2/tsdf_server.hpp>

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
    sub_point_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/point_cloud", 1, std::bind(&TsdfServer::pointCloudCallback, this, std::placeholders::_1)
    );
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

        // ToDo convert to Transform Matrix
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10, "%s", ex.what());
    }
}

}