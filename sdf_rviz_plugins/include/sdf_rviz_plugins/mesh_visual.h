#pragma once

#include <limits>
#include <map>
#include <string>

#include <OgreManualObject.h>

#include <sdf_generator/core/block.hpp>
#include <sdf_generator/core/hash.hpp>
#include <sdf_msgs/msg/mesh.hpp>
#include <sdf_msgs/msg/multi_mesh.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <sdf_generator/mesh/util.hpp>

namespace sdf_rviz_plugins
{
class MeshVisual
{
public:
    MeshVisual(
        Ogre::SceneManager* scene_manager, 
        Ogre::SceneNode* parent_node, 
        std::string name_space = ""
    );

    virtual ~MeshVisual();

    void setMessage(
        const sdf_msgs::msg::Mesh::ConstPtr& msg
    );

    void setEnabled(bool enabled);

    void setPose(
        const Ogre::Vector3& position, const Ogre::Quaternion& orientation
    );

    void setFrameId(const std::string& frame_id) {frame_id_ = frame_id;}

    const std::string& getFrameId() const {return frame_id_;}


private:
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;

    unsigned int instance_number_;
    static unsigned int instance_counter_;
    std::string name_space_;
    bool is_enabled_;
    std::string frame_id_;

    sdf_generator::AnyIndexHashMapType<Ogre::ManualObject*>::type object_map_;
};
}