#pragma once

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/voxel.hpp>

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

template <typename VoxelType>
inline Scalar getVoxelSdf(const VoxelType& voxel);

template <>
inline Scalar getVoxelSdf(const TsdfVoxel& voxel)
{
    return voxel.distance_;
}

template <>
inline Scalar getVoxelSdf(const EsdfVoxel& voxel)
{
    return voxel.distance_;
}


}