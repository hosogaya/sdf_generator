#pragma once

#include <atomic>
#include <limits>
#include <vector>

#include <sdf_generator/core/type.hpp>

namespace sdf_generator
{
template <size_t unmasked_bits, typename StoredElement,
    typename IndexType, typename IndexTypeHasher>
class ApproxHashArray
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StoredElement& get(const size_t& hash) {return pseudo_map_[hash & bit_mask_];}
    StoredElement& get(const IndexTyep& index, size_t& hash)
    {
        hash = hasher_(index);
        return get(hash);
    }
    StoredElement& get(const IndexType& index)
    {
        size_t hash = hasher_(index);
        return get(hash);
    }

private:
    static constexpr size_t pseudo_map_size_ = (1 << unmasked_bits);
    static constexpr size_t bit_mask_ = (1 << unmasked_bits) - 1;

    std::array<StoredElement, pseudo_map_size_> pseudo_map_;
    IndexTypeHasher hasher_;
};
}