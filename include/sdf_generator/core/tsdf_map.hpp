#pragma once

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/layer.hpp>

namespace sdf_generator
{
class TsdfMap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<TsdfMap>;

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        Scalar tsdf_voxel_size_ = 0.2;
        size_t tsdf_volxels_per_side_ = 16u;

    };

    explicit TsdfMap(const Config& config)
    {

    }

protected:
    Scalar block_size_;
    Layer<TsdfVoxel>::Ptr layer_;

};
}