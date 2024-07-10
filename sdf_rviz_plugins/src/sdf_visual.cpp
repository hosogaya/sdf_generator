#include <sdf_rviz_plugins/sdf_visual.h>

namespace sdf_rviz_plugins
{
SdfVisual::SdfVisual(
    Ogre::SceneManager* scene_manager, 
    Ogre::SceneNode* parent_node, 
    std::string name_space
)
: scene_manager_(scene_manager), name_space_(name_space), is_enabled_(true)
{
    frame_node_ = parent_node->createChildSceneNode();
}

SdfVisual::~SdfVisual() {}

void SdfVisual::setMessage(
    const sdf_msgs::msg::Layer::ConstPtr& msg
)
{

}

void SdfVisual::setEnabled(bool enabled)
{
    is_enabled_ = enabled;
}
}