#include <sdf_rviz_plugins/material_loader.hpp>

#include <OGRE/OgreResourceGroupManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace sdf_rviz_plugins {

bool MaterialLoader::materials_loaded_ = false;

void MaterialLoader::loadMaterials() {
  if (materials_loaded_) {
    return;
  }
  std::string path = ament_index_cpp::get_package_share_directory("sdf_rviz_plugins")
                    + "/content/materials";

  // first instance loads a custom ogre material that supports transparent
  // colors.
  Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup(
      "SdfMaterials");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path, "FileSystem", "SdfMaterials", true);
  Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup(
      "SdfMaterials");
  Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup(
      "SdfMaterials");
  materials_loaded_ = true;
}

}  // namespace voxblox_rviz_plugin
