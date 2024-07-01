#include <sdf_generator/mesh/util.hpp>

namespace sdf_generator
{

template <typename VoxelType>
bool getSdfIfValid(
    const VoxelType& voxel, const Scalar min_weight, Scalar& sdf
)
{
    return false;
}

template <>
inline bool getSdfIfValid(
    const TsdfVoxel& voxel, const Scalar min_weight, Scalar& sdf
)
{
    if (voxel.weight_ <= min_weight) return false;
    sdf = voxel.distance_;
    return true;
}

template <>
inline bool getSdfIfValid(
    const EsdfVoxel& voxel, const Scalar /*min_weight*/,
    Scalar& sdf) {
  if (!voxel.observed_) {
    return false;
  }
  sdf = voxel.distance_;
  return true;
}

template <typename VoxelType>
bool getColorIfValid(
    const VoxelType& voxel, const Scalar min_weight, Color& color)
{
    return false;
}

template <>
inline bool getColorIfValid(
    const TsdfVoxel& voxel, const Scalar min_weight, Color& color) 
{
    if (voxel.weight_ <= min_weight) {
        return false;
    }
    color = voxel.color_;
    return true;
}

template <>
inline bool getColorIfValid(
    const EsdfVoxel& voxel, const Scalar /*min_weight*/, Color& color) 
{
    if (!voxel.observed_) {
        return false;
    }
    color = Color(255u, 255u, 255u);
    return true;
}
}