#pragma once

#include <sdf_generator/core/type.hpp>

namespace sdf_generator
{
template <typename IndexType>
inline IndexType point2voxelIndex(const Point& point, const Scalar grid_size_inv)
{
    return IndexType
    (
        std::floor(point.x()*grid_size_inv + std::numeric_limits<Scalar>::epsilon()),
        std::floor(point.y()*grid_size_inv + std::numeric_limits<Scalar>::epsilon()),
        std::floor(point.z()*grid_size_inv + std::numeric_limits<Scalar>::epsilon()),
    );
}

template <typename IndexType>
inline Point getCenterPointOfVoxel(const IndexType& index, Scalar voxel_size)
{
    return Point(
        static_cast<Scalar>(index.x() + 0.5) * voxel_size,
        static_cast<Scalar>(index.y() + 0.5) * voxel_size,
        static_cast<Scalar>(index.z() + 0.5) * voxel_size,
    );
}


}