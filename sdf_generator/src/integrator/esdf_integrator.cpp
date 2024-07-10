#include <sdf_generator/integrator/esdf_integrator.hpp>
#include <iostream>

namespace sdf_generator
{
EsdfIntegrator::EsdfIntegrator(
    const Config config, typename Layer<TsdfVoxel>::Ptr tsdf_layer, 
    typename Layer<EsdfVoxel>::Ptr esdf_layer
)
: config_(config), tsdf_layer_(tsdf_layer), esdf_layer_(esdf_layer)
{
    esdf_voxels_per_side_ = esdf_layer_->voxelsPerSide();
    esdf_voxel_size_ = esdf_layer->voxelSize();

    update_queue_.setNumBuckets(config_.num_buckets_, config_.default_distance_);
}

void EsdfIntegrator::updateFromTsdfLayer(bool clear_updated_flag)
{
    BlockIndexList tsdf_blocks;
    tsdf_layer_->getAllUpdatedBlocks(Update::kEsdf, tsdf_blocks);

    if (tsdf_blocks.size() > 0)
    {
        updateFromTsdfBlocks(tsdf_blocks);

        if (clear_updated_flag)
        {
            for (const auto& block_index: tsdf_blocks)
            {
                if (tsdf_layer_->hasBlock(block_index))
                {
                    tsdf_layer_->getBlockPtr(block_index)->setUpdated(Update::kEsdf, false);
                }
            }
        }
    }
}

void EsdfIntegrator::updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks)
{
    for (const auto& block_index: tsdf_blocks)
    {
        auto tsdf_block_ptr = tsdf_layer_->getBlockPtr(block_index);
        if (!tsdf_block_ptr) continue;

        // Allocate the same block in the ESDF layer.
        // Block indices are the same across all layers.
        Block<EsdfVoxel>::Ptr esdf_block_ptr = esdf_layer_->getBlockPtrWithAllocation(block_index);
        esdf_block_ptr->setUpdatedAll();

        const size_t num_voxels_per_block = tsdf_block_ptr->numVoxels();
        for (size_t linear_index=0; linear_index<num_voxels_per_block; ++linear_index)
        {
            TsdfVoxel& tsdf_voxel = tsdf_block_ptr->getVoxel(linear_index);
            
            // If this voxel is unobserved in the original map, skip it
            if (!isObserved(tsdf_voxel.weight_)) continue;

            EsdfVoxel& esdf_voxel = esdf_block_ptr->getVoxel(linear_index);
            esdf_voxel.behind_ = tsdf_voxel.distance_ < 0.0;

            bool current_occupied;

            if (config_.finer_esdf_on_)
                current_occupied = isOccupied(tsdf_voxel.distance_, tsdf_voxel.gradient_);
            else
                current_occupied = isOccupied(tsdf_voxel.distance_);

            if (esdf_voxel.self_idx_(0) == UNDEF)
            {   
                // not yet initialized
                esdf_voxel.observed_ = true;
                esdf_voxel.newly_ = true;
                VoxelIndex voxel_index = esdf_block_ptr->calVoxelIndex(linear_index);
                GlobalIndex global_index = calGlobalVoxelIndex(block_index, voxel_index, esdf_layer_->voxelsPerSide());
                esdf_voxel.self_idx_ = global_index;

                if (esdf_voxel.behind_) esdf_voxel.raw_distance_ = -config_.max_behind_surface_;
                else esdf_voxel.raw_distance_ = config_.default_distance_;

                // Newly found occupied --> insert list
                if (current_occupied) insert_list_.push_back(global_index);
            }
            else
            {
                esdf_voxel.newly_ = false;

                // Originally occupied but not occupied --> delete
                if (tsdf_voxel.occupied_ && !current_occupied)
                    delete_list_.push_back(esdf_voxel.self_idx_);
                if (!tsdf_voxel.occupied_ && current_occupied)
                    insert_list_.push_back(esdf_voxel.self_idx_);
            }

            tsdf_voxel.occupied_ = current_occupied;

            const bool tsdf_fixed = isFixed(tsdf_voxel.distance_);
            if (config_.fixed_band_esdf_on_ && tsdf_fixed)
            {
                // In fixed band, initialize with the current tsdf.
                esdf_voxel.distance_ = tsdf_voxel.distance_;
                esdf_voxel.fixed_ = true;
            }
            else
            {
                esdf_voxel.fixed_ = false;
            }
        }
    }

    if (insert_list_.size() + delete_list_.size() > 0) 
    {
        getUpdateRange();
        setLocalRange();
        updateESDF();
    }
}

void EsdfIntegrator::getUpdateRange()
{
    update_range_min_ << UNDEF, UNDEF, UNDEF;
    update_range_max_ << -UNDEF, -UNDEF, -UNDEF;

    for (const auto& index: insert_list_)
    {
        for (int j=0; j<3; ++j)
        {
            update_range_min_(j) = std::min(index(j), update_range_min_(j));
            update_range_max_(j) = std::max(index(j), update_range_max_(j));
        }
    }

    for (const auto& index: delete_list_)
    {
        for (int j=0; j<3; ++j)
        {
            update_range_min_(j) = std::min(index(j), update_range_min_(j));
            update_range_max_(j) = std::max(index(j), update_range_max_(j));
        }
    }
}



// Expand the updated range with a given margin and then allocate memory
void EsdfIntegrator::setLocalRange()
{
    range_min_ = update_range_min_ - config_.range_boundary_offset_;
    range_max_ = update_range_max_ + config_.range_boundary_offset_;

    // Allocate memory for the local Esdf map
    BlockIndex block_range_min, block_range_max;
    for (int i=0; i<3; ++i)
    {
        block_range_min(i) = range_min_(i)/esdf_voxels_per_side_;
        block_range_max(i) = range_max_(i)/esdf_voxels_per_side_;
    }

    for (int x=block_range_min(0); x <= block_range_max(0); ++x)
    {
        for (int y=block_range_min(1); y <= block_range_max(1); ++y)
        {
            for (int z=block_range_min(2); z <= block_range_max(2); ++z)
            {
                BlockIndex cur_block_index(x, y, z);
                //Allocate Esdf Block if it hasn't been allocated
                auto esdf_block_ptr = esdf_layer_->getBlockPtrWithAllocation(cur_block_index);
                esdf_block_ptr->setUpdatedAll();

                // Allocate Tsdf Block
                if (config_.allocate_tsdf_in_range_)
                {
                    auto tsdf_block_ptr = tsdf_layer_->getBlockPtrWithAllocation(cur_block_index);
                }
            }
        }
    }
}

// set all the voxels in the range to be unfixed
void EsdfIntegrator::resetFixed()
{
    for (int x=range_min_(0); x <= range_max_(0); ++x)
    {
        for (int y=range_min_(1); y <= range_max_(1); ++y)
        {
            for (int z=range_min_(2); z <= range_max_(2); ++z)
            {
                GlobalIndex cur_voxel_index(x, y, z);
                EsdfVoxel* voxel = esdf_layer_->getVoxelPtr(cur_voxel_index);
                voxel->fixed_ = false;
            }
        }
    }
}


/* Delete idx from the doubly linked list
 * input:
 * occ_vox: head voxel of the list
 * cur_vox: the voxel need to be deleted
 */
void EsdfIntegrator::deleteFromList(
    EsdfVoxel* occ_voxel, EsdfVoxel* cur_voxel
)
{
    if(cur_voxel->prev_idx_(0) != UNDEF)
    {
        EsdfVoxel* prev_voxel = esdf_layer_->getVoxelPtr(cur_voxel->prev_idx_);
        prev_voxel->next_idx_ = cur_voxel->next_idx_;
    }
    else 
    {
        occ_voxel->head_idx_ = cur_voxel->next_idx_;
    }

    if (cur_voxel->next_idx_(0) != UNDEF)
    {
        EsdfVoxel* next_voxel = esdf_layer_->getVoxelPtr(cur_voxel->next_idx_);
        next_voxel->prev_idx_ = cur_voxel->prev_idx_;
    }

    cur_voxel->next_idx_ << UNDEF, UNDEF, UNDEF;
    cur_voxel->prev_idx_ << UNDEF, UNDEF, UNDEF;
}

/* Insert idx to the doubly linked list at the head
 * input:
 * occ_vox: head voxel of the list
 * cur_vox: the voxel need to be insert
 */
void EsdfIntegrator::insertIntoList(
    EsdfVoxel* occ_voxel, EsdfVoxel* cur_voxel
)
{
    if (occ_voxel->head_idx_(0) == UNDEF)
    {
        occ_voxel->head_idx_ = cur_voxel->self_idx_;
    }
    else 
    {
        EsdfVoxel* head_occ_voxel = esdf_layer_->getVoxelPtr(occ_voxel->head_idx_);

        head_occ_voxel->prev_idx_ = cur_voxel->self_idx_;
        cur_voxel->next_idx_ = occ_voxel->head_idx_;
        occ_voxel->head_idx_ = cur_voxel->self_idx_;
    }
}

/**
 * Intuitively, Voxfield is a combination of Voxblox and FIESTA
 * On one hand, it uses the efficient incremental updating idea from FIESTA
 * (book keeping with doubly linked list).
 * On the other hand, it uses the underlying data structure of Voxblox and
 * also update ESDF map from TSDF map instead of occupancy grids map like
 * Voxblox so that the distance from the occupied voxel's center to the object
 * surface is also take into account. The discretization error present in FIESTA
 * and EDT can be avoided. Besides, unlike Voxblox, the true Euclidean distance
 * is adopted.
 */
void EsdfIntegrator::updateESDF()
{
    /**
     * update_queue_ is a priority queue, voxels with the smaller absolute
     * distance would be updated first once a voxel is added to update_queue_, its
     * distance would be fixed and it would act as a seed for further updating
     */

    // std::cout << "[updateESDF] insert list size: " << delete_list_.size() << std::endl;
    // initialization
    while (!insert_list_.empty())
    {
        GlobalIndex cur_voxel_index(insert_list_.front());
        insert_list_.erase(insert_list_.begin());


        // delete previous link & create a new linkded-list
        EsdfVoxel* cur_voxel = esdf_layer_->getVoxelPtr(cur_voxel_index);
        if (cur_voxel->coc_idx_(0) != UNDEF)
        {
            EsdfVoxel* coc_voxel = esdf_layer_->getVoxelPtr(cur_voxel->coc_idx_);
            deleteFromList(coc_voxel, cur_voxel);
        }

        cur_voxel->raw_distance_ = 0.0f;
        cur_voxel->coc_idx_ = cur_voxel_index;
        insertIntoList(cur_voxel, cur_voxel);
        update_queue_.push(cur_voxel_index, 0.0f);

    }

    // std::cout << "[updateESDF] delete list size: " << delete_list_.size() << std::endl;
    // increasing update
    while (!delete_list_.empty())
    {
        GlobalIndex cur_voxel_index(delete_list_.front());
        delete_list_.erase(delete_list_.begin());

        EsdfVoxel* cur_voxel = esdf_layer_->getVoxelPtr(cur_voxel_index);

        GlobalIndex next_voxel_index(UNDEF, UNDEF, UNDEF);

        // for each voxel in current voxel's doubly linkded list
        // (regard current voxel as the closest occupied voxel)
        for (GlobalIndex index = cur_voxel_index; index(0) != UNDEF; index = next_voxel_index)
        {
            EsdfVoxel* voxel = esdf_layer_->getVoxelPtr(index);

            voxel->coc_idx_ << UNDEF, UNDEF, UNDEF;
            if (voxelInRange(index))
            {
                voxel->raw_distance_ = config_.default_distance_;

                // 24 neighborhood
                Neighborhood24::IndexMatrix neighbor_voxels_index;
                Neighborhood24::getFromGlobalIndex(index, neighbor_voxels_index);

                // Go through the neighbors and see if we can update 
                // this voxels' closest occupied voxel. 
                for (unsigned int i = 0u; i < neighbor_voxels_index.cols(); ++i)
                {
                    const GlobalIndex& neighbor_voxel_index = neighbor_voxels_index.col(i);
                    if (!voxelInRange(neighbor_voxel_index)) continue;

                    EsdfVoxel* neighbor_voxel = esdf_layer_->getVoxelPtr(neighbor_voxel_index);

                    const GlobalIndex& neighbor_coc_voxel_index = neighbor_voxel->coc_idx_;

                    if (!neighbor_voxel->observed_ || neighbor_coc_voxel_index(0) ==UNDEF) continue;
                    
                    TsdfVoxel* neighbor_coc_tsdf_voxel = tsdf_layer_->getVoxelPtr(neighbor_coc_voxel_index);

                    if (!neighbor_coc_tsdf_voxel->occupied_) continue;
                    
                    float distance = calDistance(neighbor_coc_voxel_index, index);
                    if (distance < std::abs(voxel->raw_distance_))
                    {
                        voxel->raw_distance_ = distance;
                        voxel->coc_idx_ = neighbor_coc_voxel_index;
                    }
                    if (config_.early_break_)
                    {
                        voxel->newly_ = true;
                        break;
                    }
                }
            }
            next_voxel_index = voxel->prev_idx_;
            voxel->next_idx_ << UNDEF, UNDEF, UNDEF;
            voxel->prev_idx_ << UNDEF, UNDEF, UNDEF;

            if (voxel->coc_idx_(0) == UNDEF) continue;

            if (voxel->behind_) voxel->raw_distance_ *= -1.0f;
            
            update_queue_.push(index, voxel->raw_distance_);
            EsdfVoxel* coc_voxel = esdf_layer_->getVoxelPtr(voxel->coc_idx_);
            insertIntoList(coc_voxel, voxel);
        }
        cur_voxel->head_idx_ << UNDEF, UNDEF, UNDEF;
    }

    // std::cout << "[updateESDF] update_queue size: " << update_queue_.size() << std::endl;
    // Esdf decreasing unupdating (BFS based on priority queue) 
    int updated_count = 0;
    int patch_count = 0;
    while (!update_queue_.empty())
    {   
        const GlobalIndex cur_voxel_index(update_queue_.front());
        update_queue_.pop();
        EsdfVoxel* cur_voxel = esdf_layer_->getVoxelPtr(cur_voxel_index);

        EsdfVoxel* coc_voxel = esdf_layer_->getVoxelPtr(cur_voxel->coc_idx_);

        // if the voxel lies in the fixed band, then default value is its tsdf
        // if out, apply the finer esdf correction, add the sub-voxel part
        // of the esdf from the voxel center to the actual surface
        if (config_.finer_esdf_on_)
        {
            // out of the fixed band
            if (!cur_voxel->fixed_)
            {
                TsdfVoxel* coc_tsdf_voxel = tsdf_layer_->getVoxelPtr(cur_voxel->coc_idx_);
                if (coc_tsdf_voxel->gradient_.norm() > kEpsilon
                && coc_tsdf_voxel->occupied_)
                {
                    // gurantee the gradient here is a unit vector
                    coc_tsdf_voxel->gradient_.normalize();
                    Point cur_voxel_center = cur_voxel_index.cast<Scalar>()*esdf_voxel_size_;
                    Point coc_voxel_center = cur_voxel->coc_idx_.cast<Scalar>()*esdf_voxel_size_;
                    // gradient is pointing toward the sensor, sign should be positive
                    Point coc_voxel_surface = coc_voxel_center + config_.gradient_sign_
                                                                *coc_tsdf_voxel->gradient_
                                                                *coc_tsdf_voxel->distance_;
                    
                    cur_voxel->distance_ = (coc_voxel_surface - cur_voxel_center).norm();
                    if (cur_voxel->behind_) cur_voxel->distance_ *= -1.0f;
                }
                else cur_voxel->distance_ = cur_voxel->raw_distance_;
            }
        }
        else 
        {
            // use the original voxel center to center distance
            if (!cur_voxel->fixed_ || !config_.fixed_band_esdf_on_)
                cur_voxel->distance_ = cur_voxel->raw_distance_;
        }
        ++updated_count;
        ++total_updated_count_;

        // Get the global indices of neighbors
        Neighborhood24::IndexMatrix neighbor_voxels_index;
        Neighborhood24::getFromGlobalIndex(cur_voxel_index, neighbor_voxels_index);

        if (config_.patch_on_ && cur_voxel->newly_)
        {
            // only newly added voxels are required for checkig
            cur_voxel->newly_ = false;
            bool change_flag = false;
            // Go throught the neighbors  and see if we need to update the cur_voxel
            for (unsigned int i=0; i<neighbor_voxels_index.cols(); ++i)
            {
                const GlobalIndex& neighbor_voxel_index = neighbor_voxels_index.col(i);
                if (!voxelInRange(neighbor_voxel_index)) continue; // for i
                
                EsdfVoxel* neighbor_voxel = esdf_layer_->getVoxelPtr(neighbor_voxel_index);
                if (!neighbor_voxel->observed_ || neighbor_voxel->coc_idx_(0) == UNDEF) continue; // for i

                Scalar distance = calDistance(neighbor_voxel->coc_idx_, cur_voxel_index);
                if (distance < std::abs(cur_voxel->raw_distance_))
                {
                    cur_voxel->raw_distance_ = distance;
                    cur_voxel->coc_idx_ = neighbor_voxel->coc_idx_;
                    change_flag = true;
                }
            }
            if (change_flag)
            {
                if (cur_voxel->behind_) cur_voxel->raw_distance_ *= -1.0f;
                deleteFromList(coc_voxel, cur_voxel);
                coc_voxel = esdf_layer_->getVoxelPtr(cur_voxel->coc_idx_);
                update_queue_.push(cur_voxel_index, cur_voxel->raw_distance_);
                insertIntoList(coc_voxel, cur_voxel);
                ++patch_count;
                continue; // while (!update_queue.empty())
            }
        } // end patch

        // updated
        std::vector<int> used_neighbor_index;
        Neighborhood24::getFromGlobalIndexAndObstacle(
            cur_voxel->self_idx_, 
            cur_voxel->coc_idx_, 
            used_neighbor_index
        );
        
        // faster version, expand only a subset of the neighborhood
        // according to the relative orientation of the closest
        // occupied voxel
        for (unsigned int i=0; i<used_neighbor_index.size(); ++i)
        {
            const GlobalIndex& neighbor_voxel_index = neighbor_voxels_index.col(used_neighbor_index[i]);

            // Check if this index in the range and not updated yet
            if (!voxelInRange(neighbor_voxel_index)) continue;

            EsdfVoxel* neighbor_voxel = esdf_layer_->getVoxelPtr(neighbor_voxel_index);
            if (!neighbor_voxel->observed_ || std::abs(neighbor_voxel->raw_distance_) == 0.0f) continue;

            Scalar distance = calDistance(cur_voxel->coc_idx_, neighbor_voxel_index);
            if (distance < std::abs(neighbor_voxel->raw_distance_))
            {
                if (neighbor_voxel->behind_) neighbor_voxel->raw_distance_ = -distance;
                else neighbor_voxel->raw_distance_ = distance;

                if (neighbor_voxel->coc_idx_(0) != UNDEF)
                {
                    EsdfVoxel* neighbor_coc_voxel = esdf_layer_->getVoxelPtr(neighbor_voxel->coc_idx_);
                    deleteFromList(neighbor_coc_voxel, neighbor_voxel);
                }
                neighbor_voxel->coc_idx_ = cur_voxel->coc_idx_;
                insertIntoList(coc_voxel, neighbor_voxel);
                update_queue_.push(neighbor_voxel_index, neighbor_voxel->raw_distance_);
            }
        }
    }

    // std::cout << "[updatedESdf] update count: " << total_updated_count_ << std::endl;
}


inline float EsdfIntegrator::calDistance(
    const GlobalIndex& voxel_index1, const GlobalIndex& voxel_index2
)
{
    return (voxel_index2 - voxel_index1).cast<float>().norm()*esdf_voxel_size_;
}

inline int EsdfIntegrator::calDistanceSquare(
    const GlobalIndex& voxel_index1, const GlobalIndex& voxel_index2
)
{
    int dx = voxel_index1(0) - voxel_index2(0);
    int dy = voxel_index1(1) - voxel_index2(1);
    int dz = voxel_index1(2) - voxel_index2(2);

    return (dx*dx + dy*dy + dz*dz);
}

inline bool EsdfIntegrator::voxelInRange(const GlobalIndex& voxel_index)
{
    return (
        voxel_index(0) >= range_min_(0) && voxel_index(0) <= range_max_(0)
     && voxel_index(1) >= range_min_(1) && voxel_index(1) <= range_max_(1)
     && voxel_index(2) >= range_min_(2) && voxel_index(2) <= range_max_(2)
    );
}

void EsdfIntegrator::assignError(
    GlobalIndex vox_idx, float esdf_error
)
{
    EsdfVoxel* voxel = esdf_layer_->getVoxelPtr(vox_idx);
    voxel->error_ = esdf_error;
}

}