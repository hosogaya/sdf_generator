#pragma once

#include <thread>
#include <mutex>

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/util.hpp>
#include <sdf_generator/core/layer.hpp>
#include <sdf_generator/core/transform_matrix.hpp>
#include <sdf_generator/point_cloud/type.hpp>
#include <sdf_generator/integrator/approx_has_array.hpp>


namespace sdf_generator
{
// non projective
class TsdfIntegratorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<TsdfIntegratorBase>;
    
    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Scalar default_truncation_distance_ = 0.1;
        Scalar max_weight_ = 1.0e4;
        bool voxel_carving_enabled_ = true;
        Scalar min_ray_length_ = 0.1;
        Scalar max_ray_length_ = 5.0;
        bool use_const_weight_ = false;
        float weight_reduction_exp_ = 1.0f;
        bool allow_clear_ = true;

        bool use_weight_dropoff_ = true;
        // if negative, then the value would be the ratio of voxel size
        float weight_dropoff_epsilon_ = -1.0f;
        
        bool use_sparsity_compensation_factor_ = false;
        Scalar sparsity_compensation_factor_ = 1.0f;

        /// non-projective correction specific
        bool normal_available_ = false;
        Scalar reliable_band_ratio_ = 2.0f;
        bool curve_assumption_ = false;
        Scalar reliable_normal_ratio_thre_ = 0.1f;

        size_t integrator_threads_ = std::thread::hardware_concurrency();

        std::string integration_order_mode_ = "mixed";
        bool merge_with_clear_ = true;
        bool enable_anti_grazing_ = false;
    
        Scalar start_voxel_subsampling_factor_ = 2.0f;
        int max_consecutive_ray_collisions_ = 2;
        int clear_checks_every_n_frames_ = 1;
        Scalar max_integration_time_s_ = std::numeric_limits<Scalar>::max();
    };

    TsdfIntegratorBase(const Config& config, Layer<TsdfVoxel>::Ptr layer);
    ~TsdfIntegratorBase();

    virtual void integratePointArray(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_C, 
        const Vector3Array& normals_c, const ColorArray& colors, const bool freespace_points = false) = 0;

    const Config& getConfig() const {return config_;}

    void setLayer(Layer<TsdfVoxel>::Ptr layer);

protected:
    inline bool isPointValid(const Point& point_c, const bool freespace_point, bool& is_clearing) const
    {
        const Scalar ray_distance = point_c.norm();
        if (ray_distance < config_.min_ray_length_) return false;
        else if (ray_distance > config_.max_ray_length_)
        {
            if (config_.allow_clear_ || freespace_point)
            {
                is_clearing = true;
                return true;
            }
            else return false;
        }
        else 
        {
            is_clearing = freespace_point;
            return true;
        }
    }

    /**
     * Will return a pointer to a voxel located at global_voxel_idx in the tsdf
     * layer. Thread safe.
     * Takes in the last_block_idx and last_block to prevent unneeded map lookups.
     * If this voxel belongs to a block that has not been allocated, a block in
     * temp_block_map_ is created/accessed and a voxel from this map is returned
     * instead. Unlike the layer, accessing temp_block_map_ is controlled via a
     * mutex allowing it to grow during integration.
     * These temporary blocks can be merged into the layer later by calling
     * updateLayerWithStoredBlocks
     */
    TsdfVoxel* allocateStorageAndGetVoxelPtr(const GlobalIndex& voxel_index, Block<TsdfVoxel>::Ptr last_block, BlockIndex& last_block_index);

    /**
     * Merges temporarily stored blocks into the main layer. NOT thread safe, see
     * allocateStorageAndGetVoxelPtr for more details.
     */
    void updateLayerWithStoredBlocks();

    // updates tsdf_voxel, thread safe
    void updateTsdfVoxel(
        const TransformMatrix<Scalar>& tf_global2current, const Point& origin, 
        const Point& point_c, const Point& point_g,
        const Vector3& normal_c, const Vector3& normal_g,
        const GlobalIndex& global_voxel_index, const Color& color,
        const Scalar init_weight, TsdfVoxel& tsdf_voxel);

    void updateTsdfVoxelValue(
        TsdfVoxel& voxel, const Scalar distance, const Scalar weight,
        const Color* color = nullptr) const; 

    /// Update tsdf_voxel's signed distance gradient
    void updateTsdfVoxelGradient(
        TsdfVoxel& voxel, const Vector3 normal, const Scalar weight) const;

    Scalar calDistance(
        const Point& origin, const Point& point_g,
        const Point& voxel_center) const;

    /// Calculates measurment weight (confidence)
    Scalar calVoxelWeight(
        const Point& point_c, const Scalar distance, const bool with_init_weight,
        const Scalar init_weight) const;  

    Scalar getVoxelWeight(const Point& point_c) const;

protected:
    Config config_;
    Layer<TsdfVoxel>::Ptr layer_;

    Scalar voxel_size_;
    size_t voxels_per_side_;
    Scalar block_size_;

    Scalar voxel_size_inv_;
    Scalar voxels_per_side_inv_;
    Scalar block_size_inv_;

    std::mutex temp_block_mutex_;
    Layer<TsdfVoxel>::BlockHashMap temp_block_map_;

    ApproxHashArray<12, std::mutex, GlobalIndex, LongIndexHash> mutexes_;
};
}