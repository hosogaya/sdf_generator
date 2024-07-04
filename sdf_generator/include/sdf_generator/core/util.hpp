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

inline GlobalIndex calGlobalVoxelIndex(
    const BlockIndex& block_index, const VoxelIndex& voxel_index, int voxels_per_side
)
{
    return GlobalIndex(
        block_index.cast<LongIndexElement>()*voxels_per_side
        + voxel_index.cast<LongIndexElement>()
    );
}

// assume voxels per side is power of 2
inline VoxelIndex calLocalVoxelIndex(const GlobalIndex& global_index, const int voxels_per_side)
{
    constexpr int offset = 1 << (8 * sizeof(IndexElement) - 1);
    return VoxelIndex(
        (global_index.x() + offset) & (voxels_per_side - 1),
        (global_index.y() + offset) & (voxels_per_side - 1),
        (global_index.z() + offset) & (voxels_per_side - 1)
    );
}

inline BlockIndex calBlockIndex(
    const GlobalIndex& global_voxel_index, Scalar voxels_per_side_inv
)
{
    return BlockIndex(
        std::floor(static_cast<Scalar>(global_voxel_index.x()*voxels_per_side_inv)),
        std::floor(static_cast<Scalar>(global_voxel_index.y()*voxels_per_side_inv)),
        std::floor(static_cast<Scalar>(global_voxel_index.z()*voxels_per_side_inv))
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

template <typename IndexType>
inline Point calOrigin(const IndexType& index, Scalar grid_size)
{
    return Point(
        static_cast<Scalar>(index.x())*grid_size,
        static_cast<Scalar>(index.y())*grid_size,
        static_cast<Scalar>(index.z())*grid_size
    );
}

inline int signum(const Scalar x)
{
    if (x == 0.0f) return 0;
    else if (x < 0.0f) return -1;
    
    return 1;
}

}