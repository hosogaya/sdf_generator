#include <sdf_generator_ros/esdf_server.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace sdf_generator;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EsdfServer>());
    rclcpp::shutdown();

    return 0;
}