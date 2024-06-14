#pragma once

#include <sdf_generator/core/type.hpp>

namespace sdf_generator
{
template <typename IndexType>
inline IndexType calGridIndex(const Point& point, const Scalar grid_size_inv)
{
    return IndexType
    (
        std::floor(point.x()*grid_size_inv + kCoordinateEpsilon),
        std::floor(point.y()*grid_size_inv + kCoordinateEpsilon),
        std::floor(point.z()*grid_size_inv + kCoordinateEpsilon)
    );
}

template <typename IndexType>
inline IndexType calGridIndex(const Point& scaled_point)
{
    return IndexType
    (
        std::floor(scaled_point.x() + kCoordinateEpsilon),
        std::floor(scaled_point.y() + kCoordinateEpsilon),
        std::floor(scaled_point.z() + kCoordinateEpsilon)
    );
}

template <typename IndexType>
inline Point calCenterPoint(const IndexType& index, Scalar voxel_size)
{
    return Point(
        static_cast<Scalar>(index.x() + 0.5) * voxel_size,
        static_cast<Scalar>(index.y() + 0.5) * voxel_size,
        static_cast<Scalar>(index.z() + 0.5) * voxel_size
    );
}

inline BlockIndex calBlockIndex(const GlobalIndex& global_voxel_index, Scalar voxels_per_side_inv)
{
    return BlockIndex(
        std::floor(static_cast<Scalar>(global_voxel_index.x())*voxels_per_side_inv), 
        std::floor(static_cast<Scalar>(global_voxel_index.y())*voxels_per_side_inv), 
        std::floor(static_cast<Scalar>(global_voxel_index.z())*voxels_per_side_inv) 
    );
}

template <typename IndexType>
inline Point calOrigin(const IndexType& index, Scalar grid_size)
{
    return Point(
        static_cast<Scalar>(index.x())*grid_size,
        static_cast<Scalar>(index.y())*grid_size,
        static_cast<Scalar>(index.z())*grid_size
    );
}

inline VoxelIndex calLocalVoxelIndex(const GlobalIndex& voxel_index, const int voxels_per_side)
{
    constexpr int offset = 1 << (8*sizeof(IndexElement) - 1);
    return VoxelIndex(
        (voxel_index.x() + offset) & (voxels_per_side - 1),
        (voxel_index.y() + offset) & (voxels_per_side - 1),
        (voxel_index.z() + offset) & (voxels_per_side - 1)
    );
}

inline int signum(const Scalar x)
{
    if (x == 0.0f) return 0;
    else if (x < 0.0f) return -1;
    
    return 1;
}

}