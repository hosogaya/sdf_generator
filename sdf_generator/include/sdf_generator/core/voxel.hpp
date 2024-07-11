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
    // distance with consdering inner distance
    Scalar distance_ = 0.0;
    // distance without inner distance
    Scalar raw_distance_ = 0.0;
    bool observed_ = false;

    Vector3 gradient_ = Vector3::Zero();

    // Whether the ESDF value is fixed as the same value of the colocated TSDF
    bool fixed_ = false;

    
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