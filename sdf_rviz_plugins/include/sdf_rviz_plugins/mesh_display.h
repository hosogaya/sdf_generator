#pragma once

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <sdf_msgs/msg/mesh.hpp>
#include <sdf_rviz_plugins/mesh_visual.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/visualization_manager.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sdf_rviz_plugins/material_loader.h>


namespace sdf_rviz_plugins
{
class MeshDisplay: public rviz_common::MessageFilterDisplay<sdf_msgs::msg::Mesh>
{
    Q_OBJECT
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MeshDisplay();
    virtual ~MeshDisplay();

public:
    void onInitialize() override;
    void reset() override;
    // set frame id

protected:
    void processMessage(sdf_msgs::msg::Mesh::ConstPtr msg) override;
    bool updateTransform(const rclcpp::Time& stamp);
    
    std::shared_ptr<rviz_common::properties::BoolProperty> visible_property_;
    std::unique_ptr<MeshVisual> visual_;

    Q_SLOT void visibleSlot();
};
}