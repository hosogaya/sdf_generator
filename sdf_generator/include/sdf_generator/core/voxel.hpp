#pragma once

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/color.hpp>

namespace sdf_generator
{
struct TsdfVoxel
{
    Scalar distance_ = 0.0;
    Scalar weight_ = 0.0;
    Ray gradient_ = Ray::Zero();
    Color color_;
    bool updated_ = false;
    bool ray_through_ = true;
    size_t ray_through_step_num_ = 0;
    bool occupied_ = false;
    bool isBehind() const {return distance_ < 0.0;}
};

inline std::ostream& operator<<(std::ostream& os, const TsdfVoxel& voxel)
{
    os << "distance: " << voxel.distance_ << ", "
       << "weight: " << voxel.weight_ << ", "
       << "gradient: " << voxel.gradient_.transpose() << ", "
       << "color: " << voxel.color_ << ", "
       << "occupied: " << voxel.occupied_ << ", "
       << "is behind: " << voxel.isBehind();

    return os;
}


#define INF FLT_MAX
#define UNDEF INT_MAX

struct EsdfVoxel
{
    Scalar distance_ = 0.0;
    Scalar raw_distance_ = 0.0;
    bool observed_ = false;

    /**
     * Whether the voxel was copied from the TSDF (false) or created from a pose
     * or some other source (true). This member is not serialized!!!
     * Used mainly for path planning
     */
    bool hallucinated_ = false;
    bool in_queue_ = false;
    bool fixed_ = false;

     /**
     * Relative direction toward parent. If itself, then either uninitialized
     * or in the fixed frontier. (used only by Voxblox)
     */
    Eigen::Vector3i parent_ = Eigen::Vector3i::Zero();
    
    /**
     * Whether the voxel is behind (negative value) or in front of the surface
     * (positive value).
     * Use signed distance instead of unsigned distance in FIESTA
     * The original opensource implementation of FIESTA is unsigned
     */
    bool behind_ = false;

    // Whether the voxel is newly observed
    bool newly_ = false;

    /**
     * ESDF mapping error at this voxel
     * only for the visualization (used in mapping error evaluation)
     */
    float error_ = 0.0f;

    /**
     * Indicator for update scheduling
     * (used only for voxedt esdf integrator)
     */
    float raise_ = -1.0f;

    /**
     * Index of this voxel's closest occupied voxel
     * Used by FIESTA and Voxfield
     */
    GlobalIndex coc_idx_ = GlobalIndex(UNDEF, UNDEF, UNDEF);

    /**
     * Simplified version of a doubly linked list (prev, next, head)
     * Used by FIESTA and Voxfield
     */
    GlobalIndex prev_idx_ = GlobalIndex(UNDEF, UNDEF, UNDEF);
    GlobalIndex next_idx_ = GlobalIndex(UNDEF, UNDEF, UNDEF);
    GlobalIndex head_idx_ = GlobalIndex(UNDEF, UNDEF, UNDEF);

    // Index of this voxel itself
    GlobalIndex self_idx_ = GlobalIndex(UNDEF, UNDEF, UNDEF);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}