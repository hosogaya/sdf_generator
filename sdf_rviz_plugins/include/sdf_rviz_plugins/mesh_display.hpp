#pragma once

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <sdf_msgs/msg/mesh.hpp>
#include <sdf_rviz_plugins/mesh_visual.hpp>


namespace sdf_rviz_plugins
{
class MeshVisual;

class MeshDisplay: public rviz_common::MessageFilterDisplay<sdf_msgs::msg::Mesh>
{
    Q_OBJECT
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MeshDisplay();
    virtual ~MeshDisplay() = default;

public:
    void onInitialize() override;
    void reset() override;
    // set frame id

protected:
    void processMessage(sdf_msgs::msg::Mesh::ConstPtr msg) override;
    bool updateTransform(const rclcpp::Time& stamp);
    
    rviz_common::properties::BoolProperty visible_property_;
    std::unique_ptr<MeshVisual> visual_;

    Q_SLOT void visibleSlot();
};
}