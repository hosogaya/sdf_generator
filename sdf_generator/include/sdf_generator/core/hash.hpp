#pragma once

#include <sdf_generator/core/type.hpp>

namespace sdf_generator
{
struct AnyIndexHash
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t sl = 17191; // magic number
    static constexpr size_t sl2 = sl*sl;

    std::size_t operator()(const AnyIndex& index) const
    {
        return static_cast<std::size_t>(
            index.x() + index.y()*sl + index.z()*sl2
        );
    }
};

template <typename ValueType>
struct AnyIndexHashMapType
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using type = std::unordered_map<
                    AnyIndex, ValueType, 
                    AnyIndexHash, std::equal_to<AnyIndex>, 
                    Eigen::aligned_allocator<std::pair<const AnyIndex, ValueType>>
                >;
};

struct LongIndexHash
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr size_t sl = 17191;
    static constexpr size_t sl2 = sl*sl;

    std::size_t operator()(const LongIndex& index) const
    {
        return static_cast<unsigned int>
        (
            index.x() + index.y()*sl + index.z()*sl2
        );
    }
};

template <typename ValueType>
struct LongIndexHashMapType
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using type = std::unordered_map<
                    LongIndex, ValueType, 
                    LongIndexHash, std::equal_to<LongIndex>, 
                    Eigen::aligned_allocator<std::pair<const LongIndex, ValueType>>
                >;
};

}