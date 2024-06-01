#pragma once

#include <sdf_generator/core/block.hpp>
#include <sdf_generator/core/hash.hpp>
#include <sdf_generator/core/util.hpp>

namespace sdf_generator
{
template <typename VoxelType>
class Layer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Layer<VoxelType>>;
    using BlockType = Block<VoxelType>;
    using BlockHashMap = typename AnyIndexHashMapType<typename BlockType::Ptr>::type;
    using BlockMapPair = typename std::pair<BlockIndex, typename BlockType::Ptr>;

    Layer(Scalar voxel_size, size_t voxels_per_side)
    : voxel_size_(voxel_size), voxels_per_side_(voxels_per_side)
    {
        voxel_size_inv_ = 1.0/voxel_size_;
        voxels_per_side_inv_ = 1.0/static_cast<Scalar>(voxels_per_side);
        block_size_ = voxel_size_*voxels_per_side_;
        block_size_inv_ = 1.0/block_size_;
    }
    virtual ~Layer() {}

    inline typename BlockType::ConstPtr getBlockPtr(const BlockIndex& index) const
    {
        typename BlockHashMap::const_iterator itr = block_map_.find(index);
        if (itr != block_map_.end())
        {
            return itr->second;
        }
        else
        {
            return typename BlockType::ConstPtr();
        }
    }

    inline typename BlockType::Ptr getBlockPtr(const BlockIndex& index) 
    {
        typename BlockHashMap::const_iterator itr = block_map_.find(index);
        if (itr != block_map_.end())
        {
            return itr->second;
        }
        else
        {
            return typename BlockType::ConstPtr();
        }
    }

    inline BlockIndex getBlockIndex(const Point& coords) const
    {
        return calGridIndex<BlockIndex>(coords, block_size_inv_);
    }


    inline typename BlockType::ConstPtr getBlockPtr(const Point& coords) const
    {
        return getBlockPtr(getBlockIndex(coords));
    }

    inline void insertBlock(const std::pair<const BlockIndex, typename Block<VoxelType>::Ptr>& block_pair)
    {
        auto insert_status = block_map_.insert(block_pair);
    }

    void getAllAllocatedBlocks(BlockIndexList& blocks) const
    {
        blocks.clear();
        blocks.reserve(block_map_.size());
        for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv: block_map_)
        {
            blocks.emplace_back(kv.first);
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