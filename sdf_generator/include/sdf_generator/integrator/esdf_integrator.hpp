#pragma once

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include <sdf_generator/core/layer.hpp>
#include <sdf_generator/core/voxel.hpp>
#include <sdf_generator/integrator/bucket_queue.hpp>
#include <sdf_generator/integrator/neighbor_tools_24.hpp>

namespace sdf_generator
{
/**
 * Builds an ESDF layer out of a given TSDF layer efficiently.
 * For a description of this algorithm, please check the paper of Voxfield.
 */
class EsdfIntegrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Scalar max_distance_ = 10.0f;
        Scalar default_distance_ = 10.0f;
        Scalar max_behind_surface_ = 1.0f;
        Scalar band_distance_ = 1.0f;
        /**
         * When judge a voxel is occupied or not,
         * the threshold of TSDF distance is occ_voxel_size_ratio * voxel size
         */
        Scalar occ_voxel_size_ratio_ = 0.865f;

        Scalar min_weight_ = 1e-6;

        // Number of buckets for the bucketed priority queue.
        int num_buckets_ = 20;
        // Number of the neighbor voxels (select from 6, 18, 24 and 26)
        int num_neighbor_ = 24;

        /**
         * About the patch_on and early_break settings:
         * Fastest operation can be achieved by setting patch_on=false and
         * early_break =true. Highest accuracy can be achieved with patch_on=true
         * and early_break= false. Please set them in the config file wisely. Turn
         * on the patch code or not
         */
        bool patch_on_ = true;
        // Early break the increasing update or not
        bool early_break_ = true;

        /**
         * Use finer ESDF with the consideration of the inner voxel distance
         * from the voxel center to the actual surface.
         * the non-projective TSDF and signed distance gradient are needed
         * for the calculation.
         */
        bool finer_esdf_on_ = false;

        // use a fixed band for esdf to directly copy the tsdf value
        bool fixed_band_esdf_on_ = false;

        // sign (direction) of the gradient, towards or opposite to the surface,
        // select from 1.0 or -1.0
        float gradient_sign_ = 1.0f;

        bool allocate_tsdf_in_range_ = false;

        // Local map boundary size (unit: voxel)
        GlobalIndex range_boundary_offset_ = GlobalIndex(10, 10, 5);
    };


    EsdfIntegrator(
        const Config config, typename Layer<TsdfVoxel>::Ptr tsdf_layer, 
        typename Layer<EsdfVoxel>::Ptr esdf_layer
    );

    void updateFromTsdfLayer(bool clear_updated_flag);
    void updateFromTsdfBlocks(const BlockIndexList& tsdf_blocks);

    /**
   * In Voxfield we set a range for the ESDF update.
    * This range is expanded from the bounding box of the TSDF voxels
    * get updated during the last time interval
    */
    // Get the range of the updated tsdf grid (inserted or deleted)
    void getUpdateRange();
    // Expand the updated range with a given margin and then allocate memory
    void setLocalRange();
    // Set all the voxels in the range to be unfixed
    void resetFixed();
    // Judge if a voxel is in the update range, if not, leave it still
    inline bool voxelInRange(const GlobalIndex& vox_idx);

    // main ESDF updating function
    void updateESDF();

    // basic operations of a doubly linked list
    // delete operation
    void deleteFromList(EsdfVoxel* occ_vox, EsdfVoxel* cur_vox);
    // insert operation
    void insertIntoList(EsdfVoxel* occ_vox, EsdfVoxel* cur_vox);

    // calculate distance between two voxel centers
    inline Scalar calDistance(const GlobalIndex& vox_idx_a, const GlobalIndex& vox_idx_b);
    inline int calDistanceSquare(const GlobalIndex& vox_idx_a, const GlobalIndex& vox_idx_b);

    // Assign the ESDF mapping error of the voxel (used for evaluation)
    void assignError(GlobalIndex vox_idx, float esdf_error);


    inline void clear()
    {
        GlobalIndexList().swap(insert_list_);
        GlobalIndexList().swap(delete_list_);
        update_queue_.clear();
        updated_voxel_.clear();
    }

    float getEsdfMaxDistance() const {return config_.max_distance_;}

    void setEsdfMaxDistance(float max_distance) 
    {
        config_.max_distance_ = max_distance;
        if (config_.default_distance_ < max_distance) 
            config_.default_distance_ = max_distance;
    }

    inline bool isFixed(Scalar distance) const 
    {
        return std::abs(distance) < config_.band_distance_;
    }

    inline bool isOccupied(Scalar distance) const
    {
        return std::abs(distance) <= config_.occ_voxel_size_ratio_*esdf_voxel_size_;
    }

    inline bool isOccupied(Scalar distance, Vector3& gradient) const 
    {
        if (gradient.norm() > kEpsilon)
        {
            Vector3 dist_on_axis = distance*gradient;
            bool is_occupied = true;
            for (int i=0; i<3; ++i)
            {
                if (std::abs(dist_on_axis(i)) > 0.5f*esdf_voxel_size_)
                    is_occupied = false;
            }
            return is_occupied;
        }
        else return isOccupied(distance);
    }

    inline bool isObserved(Scalar weight) const 
    {
        return weight >= config_.min_weight_;
    }

protected:
    Config config_;

    size_t esdf_voxels_per_side_;
    Scalar esdf_voxel_size_;

    typename Layer<TsdfVoxel>::Ptr tsdf_layer_;
    typename Layer<EsdfVoxel>::Ptr esdf_layer_;

    // Data structure used for Voxfield
    GlobalIndexList insert_list_;
    GlobalIndexList delete_list_;
    BucketQueue<GlobalIndex> update_queue_;
    LongIndexSet updated_voxel_;


    // Update (inseted and deleted occupied voxels) range, unit: voxel
    GlobalIndex update_range_min_;
    GlobalIndex update_range_max_;

    // Local map range (update range + boundary size), unit: voxel
    GlobalIndex range_min_;
    GlobalIndex range_max_;

    // for recording and logging
    int total_updated_count_ = 0;
};
}