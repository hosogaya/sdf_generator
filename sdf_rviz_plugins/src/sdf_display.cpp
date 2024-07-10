#include <sdf_rviz_plugins/sdf_display.h>

namespace sdf_rviz_plugins
{

SdfDisplay::SdfDisplay()
{
    visible_property_ = std::make_shared<rviz_common::properties::BoolProperty>(
        "Visible", true,
        "Show or hide the sdf. If the sdf is hidden but not disabled, it "
        "will persist and is incrementally built in the background.",
        this, SLOT(visibleSlot())
    );

    height_property_ = std::make_shared<rviz_common::properties::FloatProperty>(
        "Height", 0.0, 
        "Extract and show the 2D SDF at this height",
        this, SLOT(heightSlot())
    );

    color_mode_property_ = std::make_shared<rviz_common::properties::EnumProperty>(
        "ColorMode", "Rainbow", 
        "Select the transformer to use to set the color", 
        this, SLOT(colorModeSlot())
    );

    color_mode_property_->addOption("Rainbow", 0);
    color_mode_property_->addOption("Gray", 1);

    color_source_property_ = std::make_shared<rviz_common::properties::EnumProperty>(
        "ColorSource", "Distance", 
        "Select the source of the color", 
        this, SLOT(colorSourceSlot())
    );
    color_source_property_->addOption("Distance", 0);
    color_source_property_->addOption("Gradient", 1);
}

SdfDisplay::~SdfDisplay() {}

void SdfDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void SdfDisplay::reset()
{
    MFDClass::reset();
    visual_.reset();
}

void SdfDisplay::processMessage(sdf_msgs::msg::Layer::ConstPtr msg)
{
    if (!visual_)
    {
        std::cout << "[SdfDisplay::processMessage] creating SdfVisual instance" << std::endl;
        visual_ = std::make_unique<SdfVisual>(context_->getSceneManager(), scene_node_);
        visibleSlot();
        heightSlot();
        colorModeSlot();
        colorSourceSlot();
    }

    visual_->setFrameId(msg->header.frame_id);
    if (updateTransform(msg->header.stamp))
        visual_->setMessage(msg);
}

bool SdfDisplay::updateTransform(const rclcpp::Time& stamp)
{
    if (!visual_) return false;

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(
        visual_->getFrameId(), stamp, position, orientation
    ))
    {
        // warning
        std::cout << "[MeshDisplay::updateTransfrom] Failed to update transform" << std::endl;
        return false;
    }

    visual_->setPose(position, orientation);

    return true;
}

void SdfDisplay::visibleSlot()
{
    if (visual_)
    {
        visual_->setEnabled(visible_property_->getBool());
        if (visible_property_->getBool())
        {
            updateTransform(context_->getClock()->now());
        }
    }
}


void SdfDisplay::heightSlot()
{
    if (visual_)
    {
        visual_->setHeight(height_property_->getFloat());
    }
}

void SdfDisplay::colorModeSlot()
{
    if (visual_)
    {
        int color_mode = color_mode_property_->getOptionInt();
        if (color_mode == 0)
            visual_->setColorMode(SdfVisual::ColorMode::Rainbow);
        else if (color_mode == 1)
            visual_->setColorMode(SdfVisual::ColorMode::Gray);
    }
}

void SdfDisplay::colorSourceSlot()
{
    if (visual_)
    {
        int color_source = color_source_property_->getOptionInt();
        if (color_source == 0)
            visual_->setColorSource(SdfVisual::ColorSource::Distance);
        else if (color_source == 1)
            visual_->setColorSource(SdfVisual::ColorSource::Gradient);
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sdf_rviz_plugins::SdfDisplay, rviz_common::Display)