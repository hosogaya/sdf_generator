#pragma once

#include <OgreResourceGroupManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace sdf_rviz_plugins
{
/**
 * This class simply loads custom ogre materials for visualization upon startup.
 */
class MaterialLoader {
 public:
  static void loadMaterials();

 private:
  MaterialLoader() = default;

  static bool materials_loaded_;
};
}