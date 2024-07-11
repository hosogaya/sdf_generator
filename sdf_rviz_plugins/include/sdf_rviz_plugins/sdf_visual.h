#pragma once

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <rviz_common/uniform_string_stream.hpp>

#include <sdf_msgs/msg/esdf_layer.hpp>
#include <sdf_generator_ros/msg_conversion.hpp>
#include <sdf_generator/point_cloud/color_maps/rainbow_color_map.hpp>
#include <sdf_generator/point_cloud/color_maps/gray_scale_color_map.hpp>

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
        Gradient_X,
        Gradient_Y,
        Gradient_Z
    };


    SdfVisual(
        Ogre::SceneManager* scene_manager, 
        Ogre::SceneNode* parent_node, 
        std::string name_space = ""
    );

    virtual ~SdfVisual();

    void setMessage(
        const sdf_msgs::msg::EsdfLayer::ConstPtr& msg
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
    void setColorMode(const SdfVisual::ColorMode& mode) 
    {
        if (!color_map_ || color_mode_ != mode) 
        {
            if (mode == ColorMode::Rainbow) color_map_ = std::make_shared<sdf_generator::RainbowColorMap>();
            else if (mode == ColorMode::Gray) color_map_ = std::make_shared<sdf_generator::GrayScaleColorMap>();
        }
        color_mode_ = mode;
    }
    void setColorSource(const SdfVisual::ColorSource& source) {color_source_ = source;}

    std::string getFrameId() const {return frame_id_;}

private:
    size_t getCols(const sdf_generator::BlockIndexList& blocks, const size_t voxels_per_side, const int height_index) const; 
    size_t getRows(const sdf_generator::BlockIndexList& blocks, const size_t voxels_per_side, const int height_index) const; 
    Ogre::ColourValue calColorValue(const sdf_generator::EsdfVoxel& voxel);

    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;

    std::string name_space_;
    bool is_enabled_;
    std::string frame_id_;
    float slice_height_ = 0.0f;
    ColorMode color_mode_ = ColorMode::Rainbow; 
    ColorSource color_source_ = ColorSource::Distance;

    Ogre::ManualObject* manual_object_;
    std::string material_name_;
    Ogre::MaterialPtr material_;

    sdf_generator::ColorMap::Ptr color_map_;
};

static const Eigen::Vector3f kGradientMultiplier{1, 1e2, 1e4};

}