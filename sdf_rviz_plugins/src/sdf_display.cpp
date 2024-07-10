#include <sdf_rviz_plugins/sdf_display.h>

namespace sdf_rivz_plugins
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

void SdfDisplay::~SdfDisplay() {}

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

}

void SdfDisplay::updateTransform(const rclcpp::Time& stamp)
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
    
}


void SdfDisplay::heightSlot()
{

}

void SdfDisplay::colorModeSlot()
{
    
}

void SdfDisplay::colorSourceSlot()
{

}

}