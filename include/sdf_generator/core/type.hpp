#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>
#include <unordered_map>

namespace sdf_generator
{
using Scalar = float;
using Point = Eigen::Vector<Scalar, 3>;
using Vector3 = Eigen::Vector<Scalar, 3>;
using Ray = Eigen::Vector<Scalar, 3>;

using IndexElement = int;
using AnyIndex = Eigen::Vector<IndexElement, 3>;
using VoxelIndex = AnyIndex;
using BlockIndex = AnyIndex;

using LongIndexElement = int64_t;
using LongIndex = Eigen::Matrix<LongIndexElement, 3, 1>;
using GlobalIndex = LongIndex;

using InterpTable = Eigen::Matrix<Scalar, 8, 8>;
using InterpVector = Eigen::Matrix<Scalar, 1, 8>;
using InterpIndexes = Eigen::Array<IndexElement, 3, 8>;


// https://qiita.com/vs4sh/items/2032fd1a5dab780d432c
// if using STL of Eigen Eigen::aligned_allocator is recomended. 
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
// substitute for make_shared
template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) 
{
    using TypeNonConst = std::remove_const<Type>::type;
    return std::allocate_shared<Type>(
            Eigen::aligned_allocator<TypeNonConst>(),
            std::forward<Arguments>(arguments)...);
}


constexpr Scalar kCoordinateEpsilon = 1e-6; // used for coordinate
constexpr Scalar kWeightEpsilon = 1e-8; // used for weights

}