#include <sdf_rviz_plugins/mesh_display.h>

namespace sdf_rviz_plugins
{
MeshDisplay::MeshDisplay()
{
    MaterialLoader::loadMaterials();
    visible_property_ = std::make_shared<rviz_common::properties::BoolProperty>(
        "Visible", true,
        "Show or hide the mesh. If the mesh is hidden but not disabled, it "
        "will persist and is incrementally built in the background.",
        this, SLOT(visibleSlot())
    );
}

MeshDisplay::~MeshDisplay() {}

void MeshDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void MeshDisplay::reset()
{
    MFDClass::reset();
    visual_.reset();
}

void MeshDisplay::visibleSlot()
{
    if (visual_)
    {
        visual_->setEnabled(visible_property_->getBool());
        if (visible_property_->getBool())
        {
            // update transformation
            updateTransform(context_->getClock()->now());
        }
    }
}

void MeshDisplay::processMessage(
    sdf_msgs::msg::Mesh::ConstPtr msg
)
{
    if (!visual_)
    {
        visual_ = std::make_unique<MeshVisual>(context_->getSceneManager(), scene_node_);
        visual_->setEnabled(visible_property_->getBool());
    }

    // update the frame, pose and mesh of the visual
    visual_->setFrameId(msg->header.frame_id);
    // update transform
    if (updateTransform(msg->header.stamp))
        visual_->setMessage(msg);
}


bool MeshDisplay::updateTransform(const rclcpp::Time& stamp)
{
    if (!visual_) return false;

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(
        visual_->getFrameId(), stamp, position, orientation
    ))
    {
        // warning
        return false;
    }

    visual_->setPose(position, orientation);

    return true;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sdf_rviz_plugins::MeshDisplay, rviz_common::Display)