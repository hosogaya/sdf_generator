#pragma once

#include <sdf_generator/core/type.hpp>
namespace sdf_generator
{
using VertexIndex = size_t;
using VertexIndexList = AlignedVector<VertexIndex>;
using Triangle = Eigen::Matrix<Scalar, 3, 3>;
using TriangleList = AlignedVector<Triangle>;
}