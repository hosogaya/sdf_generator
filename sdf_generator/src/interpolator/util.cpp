#include <sdf_generator/interpolator/util.hpp>

namespace sdf_generator
{
template <typename VoxelType>
bool isObservedVoxel(const VoxelType&) {return false;}

template <>
bool isObservedVoxel(const TsdfVoxel& voxel)
{
    return voxel.weight_ > kWeightEpsilon;
}

template <>
bool isObservedVoxel(const EsdfVoxel& voxel)
{
    return voxel.observed_;
}

}