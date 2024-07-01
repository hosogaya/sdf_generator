#include <sdf_generator_ros/tsdf_server.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace sdf_generator;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TsdfServer>());
    rclcpp::shutdown();

    return 0;
}