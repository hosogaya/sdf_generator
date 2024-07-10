#pragma once

#include <memory>
#include <string>

#include <sdf_generator/core/esdf_map.hpp>
#include <sdf_generator/core/tsdf_map.hpp>
#include <sdf_generator/integrator/esdf_integrator.hpp>
#include <sdf_generator/integrator/tsdf_integrator.hpp>
#include <sdf_generator_ros/tsdf_server.hpp>

namespace sdf_generator
{
class EsdfServer: public TsdfServer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EsdfServer(const rclcpp::NodeOptions options=rclcpp::NodeOptions().use_intra_process_comms(true));

    virtual ~EsdfServer();

protected:
    virtual void layerTimerCallback() override;

    std::shared_ptr<EsdfMap> esdf_map_;
    std::unique_ptr<EsdfIntegrator> esdf_integrator_;

    bool publish_esdf_layer_ = true;
};
}