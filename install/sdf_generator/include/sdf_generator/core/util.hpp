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

}