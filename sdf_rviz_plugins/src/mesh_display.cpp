#include <sdf_rviz_plugins/mesh_display.hpp>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz_common/visualization_manager.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sdf_rviz_plugins/material_loader.hpp>

namespace sdf_rviz_plugins
{
MeshDisplay::MeshDisplay()
: visible_property_(
          "Visible", true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          this, SLOT(visibleSlot()))
{
    MaterialLoader::loadMaterials();
}

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
        visual_->setEnabled(visible_property_.getBool());
        if (visible_property_.getBool())
        {
            // update transformation
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
        visual_->setEnabled(visible_property_.getBool());
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