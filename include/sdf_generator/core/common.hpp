#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>
#include <unordered_map>

namespace sdf_generator
{
using Scalar = float;
using Point = Eigen::Vector<Scalar, 3>;
using Ray = Eigen::Vector<Scalar, 3>;

using IndexElement = int;
using AnyIndex = Eigen::Vector<IndexElement, 3>;
using VoxelIndex = AnyIndex;


// https://qiita.com/vs4sh/items/2032fd1a5dab780d432c
// if using STL of Eigen Eigen::aligned_allocator is recomended. 
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
// substitute for make_shared
template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) 
{
  typedef typename std::remove_const<Type>::type TypeNonConst;
  return std::allocate_shared<Type>(
      Eigen::aligned_allocator<TypeNonConst>(),
      std::forward<Arguments>(arguments)...);
}

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