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

    inline typename BlockType::ConstPtr getBlockConstPtr(const BlockIndex& index) const
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
            return typename BlockType::Ptr();
        }
    }

    inline BlockIndex getBlockIndex(const Point& coords) const
    {
        return calGridIndex<BlockIndex>(coords, block_size_inv_);
    }


    inline typename BlockType::ConstPtr getBlockConstPtr(const Point& coords) const
    {
        return getBlockConstPtr(getBlockIndex(coords));
    }

    inline typename BlockType::Ptr getBlockPtr(const Point& coords) 
    {
        return getBlockPtr(getBlockIndex(coords));
    }

    inline void insertBlock(const std::pair<const BlockIndex, typename Block<VoxelType>::Ptr>& block_pair)
    {
        block_map_.insert(block_pair);
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

    void getAllUpdatedBlocks(Update::Status bit, BlockIndexList& blocks) const
    {
        blocks.clear();
        blocks.reserve(block_map_.size());
        for (const std::pair<const BlockIndex, typename BlockType::Ptr>& kv: block_map_)
        {
            if (kv.second->updated(bit)) blocks.emplace_back(kv.first);
        }
    }

    typename BlockType::Ptr allocateNewBlock(const BlockIndex& index)
    {
        auto insert_status = block_map_.emplace(
            index, std::make_shared<BlockType>(
                voxels_per_side_, voxel_size_, 
                calOrigin(index, block_size_)
            )
        );
        return insert_status.first->second;
    }

    typename BlockType::Ptr getBlockPtrWithAllocation(const BlockIndex& index)
    {
        typename BlockHashMap::iterator it = block_map_.find(index);
        if (it != block_map_.end()) return it->second;
        
        return allocateNewBlock(index);
    }

    inline VoxelType* getVoxelPtr(const GlobalIndex& global_index) 
    {
        const BlockIndex block_index = calBlockIndex(global_index, voxels_per_side_inv_);
        if (!hasBlock(block_index)) return nullptr;
        const VoxelIndex local_voxel_index = calLocalVoxelIndex(global_index, voxels_per_side_);
        typename BlockType::Ptr block_ptr = this->getBlockPtr(block_index);
        
        return &(block_ptr->getVoxel(local_voxel_index));
    }

    inline VoxelType* getVoxelConstPtr(const GlobalIndex& global_index) 
    {
        const BlockIndex block_index = calBlockIndex(global_index, voxels_per_side_inv_);
        if (!hasBlock(block_index)) return nullptr;
        const VoxelIndex local_voxel_index = calLocalVoxelIndex(global_index, voxels_per_side_);
        typename BlockType::ConstPtr block_ptr = this->getBlockConstPtr(block_index);
        
        return &(block_ptr->getConstVoxel(local_voxel_index));
    }

    bool hasBlock(const BlockIndex& block_index) const
    {
        return block_map_.count(block_index) > 0;
    }

    size_t blockNum() const {return block_map_.size();}

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