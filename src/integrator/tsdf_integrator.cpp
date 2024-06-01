#include <sdf_generator/integrator/tsdf_integrator.hpp>

namespace sdf_generator
{
TsdfIntegratorBase::TsdfIntegratorBase(const Config& config, Layer<TsdfVoxel>::Ptr layer)
: config_(config)
{
    setLayer(layer);

    if (config.integrator_threads_ == 0)
    {
        config_.integrator_threads_ = 1;
    }
    if (config_.allow_clear_ && !config_.voxel_carving_enabled)
    {
        config_.allow_clear_ = false;
    }
}

TsdfIntegratorBase::~TsdfIntegratorBase() {}

void TsdfIntegratorBase::setLayer(Layer<TsdfVoxel>::Ptr layer)
{
    layer_ = layer;

    voxel_size_ = layer_->voxelSize();
    block_size_ = layer_->blockSize();
    voxels_per_side_ = layer_->voxelsPerSide();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

TsdfVoxel* TsdfIntegratorBase::allocateStorageAndGetVoxelPtr(const GlobalIndex& voxel_index, Block<TsdfVoxel>::Ptr last_block, BlockIndex& last_block_index)
{
    const BlockIndex block_index = calBlockIndex(voxel_index, voxels_per_side_inv_);
    if ((block_index != last_block_index) || !last_block)
    {
        last_block = layer_->getBlockPtr(block_index);
        last_block_index = block_index;
    }

    if (!last_block)
    {
        std::lock_guard<std::mutex> lock(temp_block_mutex_);

        typename Layer<TsdfVoxel>::BlockHashMap::iterator itr = temp_block_map_.find(block_index);
        if (itr != temp_block_map_.end())
        {
            last_block = itr->second;
        }
        else
        {
            auto insert_status = temp_block_map_.emplace(
                block_index, std::make_shared<Block<TsdfVoxel>>(
                    voxels_per_side_, voxel_size_,
                    calOrigin(block_index, block_size_)
                )
            );

            last_block = insert_status.first->second;
        }
    }

    last_block->setUpdatedAll();

    const VoxelIndex local_voxel_index = calLocalVoxelIndex(voxel_index, voxels_per_side_);

    return &(last_block->getVoxel(local_voxel_index));   
}

void TsdfIntegratorBase::updateLayerWithStoredBlocks() 
{
    BlockIndex last_block_index;
    Block<TsdfVoxel>::Ptr block = nullptr;

    for (const std::pair<const BlockIndex, Block<TsdfVoxel>::Ptr>& block_pair: temp_block_map_)
    {
        layer_->insertBlock(block_pair);
    }

    temp_block_map_.clear();
}


}