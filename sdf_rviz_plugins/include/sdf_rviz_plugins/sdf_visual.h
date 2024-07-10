#pragma once

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <sdf_msgs/msg/layer.hpp>

namespace sdf_rviz_plugins
{

class SdfVisual
{
public:     
    enum class ColorMode
    {
        Rainbow,
        Gray
    };

    enum class ColorSource
    {
        Distance, 
        Gradient
    };


    SdfVisual(
        Ogre::SceneManager* scene_manager, 
        Ogre::SceneNode* parent_node, 
        std::string name_space = ""
    );

    virtual ~SdfVisual();

    void setMessage(
        const sdf_msgs::msg::Layer::ConstPtr& msg
    );

    void setEnabled(bool enabled);
    void setPose(
        const Ogre::Vector3& position, const Ogre::Quaternion& orientation
    )
    {
        frame_node_->setPosition(position);
        frame_node_->setOrientation(orientation);
    }
    void setFrameId(const std::string& frame_id) {frame_id_ = frame_id;}
    void setHeight(const float height) {slice_height_ = height;}
    void setColorMode(const SdfVisual::ColorMode& mode) {color_mode_ = mode;}
    void setColorSource(const SdfVisual::ColorSource& source) {color_source_ = source;}

private:
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;

    std::string name_space_;
    bool is_enabled_;
    std::string frame_id_;
    float slice_height_ = 0.0f;
    ColorMode color_mode_ = ColorMode::Rainbow; 
    ColorSource color_source_ = ColorSource::Distance;
};

}