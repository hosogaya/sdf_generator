#include <sdf_generator/core/tsdf_map.hpp>

using namespace sdf_generator;

int main()
{
    TsdfMap::Config config;
    config.tsdf_voxel_size_ = 0.2;
    config.tsdf_voxels_per_side_ = 16;

    TsdfMap map(config);
    
    return 0;
}