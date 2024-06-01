#pragma once

#include <sdf_generator/core/block.hpp>
#include <sdf_generator/core/hash.hpp>

namespace sdf_generator
{
template <typename VoxelType>
class Layer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Layer<VoxelType>>;
    using BlockType = Block<VoxelType>;
    using BlockHashMap = AnyIndexHashMapType<BlockType::Ptr>::type;
    using BlockMapPair = std::pair<BlockIndex, BlockType::Ptr>;

    Layer(Scalar voxel_size, size_t voxels_per_side)
    : voxel_size_(voxel_size), voxels_per_side_(voxels_per_side)
    {
        voxel_size_inv_ = 1.0/voxel_size;
        voxels_per_side_inv_ = 1.0/static_cast<Scalar>(voxels_per_size);
        block_size_ = voxel_size*voxels_per_side_;
        block_size_inv_ = 1.0/block_size_;
    }
    ~Layer() {}

    inline typename BlockType::ConstPtr getBlockPtrByIndex(const BlockIndex& index) const
    {
        typename BlockHashMap::const_iterator itr = block_map_.find();
        if (itr != block_map_.end())
        {
            return it->second;
        }
        else
        {
            return typename BlockType::ConstPtr();
        }
    }

    // getter
    Scalar blockSize() const {return block_size_;}
    Scalar blockSizeInv() const {return block_size_inv_;}
    Scalar voxelSize() const {return voxel_size_;}
    Scalar voxelSizeInv() const {return voxel_size_inv_;}
    size_t voxelsPerSide() const {return voxels_per_side_;}
    Scalar voxelsPerSideInv() const {return voxels_per_side_inv_;}

protected:
    Scalar voxel_size_;
    size_t voxels_per_side_;
    Scalar block_size_;

    Scalar voxel_size_inv_;
    Scalar block_size_inv_;
    Scalar voxels_per_side_inv_;

    BlockHashMap block_map_;
};
}