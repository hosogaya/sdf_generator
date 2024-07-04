#pragma once

#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>
#include <deque>
#include <queue>
#include <unordered_map>
#include <unordered_set>

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
using SignedIndex = AnyIndex;
using VoxelKey = std::pair<BlockIndex, VoxelIndex>;

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
template <typename Type>
using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedQueue = std::queue<Type, AlignedDeque<Type>>;

// substitute for make_shared
template <typename Type>
using TypeNonConst = typename std::remove_const<Type>::type;
template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) 
{
    return std::allocate_shared<Type>(
            Eigen::aligned_allocator<TypeNonConst<Type>>(),
            std::forward<Arguments>(arguments)...);
}


using IndexVector = AlignedVector<AnyIndex>;
using BlockIndexList = IndexVector;
using GlobalIndexList = AlignedVector<GlobalIndex>;

constexpr Scalar kEpsilon = 1e-8;
constexpr Scalar kCoordinateEpsilon = 1e-6; // used for coordinate
constexpr Scalar kWeightEpsilon = 1e-8; // used for weights

}