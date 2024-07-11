#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/enum_property.hpp>


#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <sdf_msgs/msg/esdf_layer.hpp>
#include <sdf_rviz_plugins/sdf_visual.h>
#include <Eigen/Core>

namespace sdf_rviz_plugins
{
class SdfDisplay: public rviz_common::MessageFilterDisplay<sdf_msgs::msg::EsdfLayer>
{
    Q_OBJECT
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SdfDisplay();
    virtual ~SdfDisplay();

    void onInitialize() override;
    void reset() override;

protected:
    void processMessage(sdf_msgs::msg::EsdfLayer::ConstPtr msg) override;
    bool updateTransform(const rclcpp::Time& stamp);

    std::shared_ptr<rviz_common::properties::BoolProperty> visible_property_;
    std::shared_ptr<rviz_common::properties::FloatProperty> height_property_;
    std::shared_ptr<rviz_common::properties::EnumProperty> color_mode_property_;
    std::shared_ptr<rviz_common::properties::EnumProperty> color_source_property_;
    std::unique_ptr<SdfVisual> visual_;

    Q_SLOT void visibleSlot();
    Q_SLOT void heightSlot();
    Q_SLOT void colorModeSlot();
    Q_SLOT void colorSourceSlot();
};

}