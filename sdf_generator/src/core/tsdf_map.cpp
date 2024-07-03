#include <sdf_generator/core/tsdf_map.hpp>

namespace sdf_generator
{
unsigned int TsdfMap::extractDistanceAndWeightOfVoxelsInSlicePlane(
    unsigned int free_plane_index, Scalar free_plane_val, 
    EigenDRef<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>>& positions,
    Eigen::Ref<Eigen::VectorX<Scalar>> distances,
    Eigen::Ref<Eigen::VectorX<Scalar>> weights, unsigned int max_points) const
{
    BlockIndexList blocks;
    layer_->getAlllocatedBlocks(blocks);

    // Cache layer settings.
    const size_t vps = layer_->voxelsPerSide();
    const size_t num_voxels_per_block = vps*vps*vps;

    bool did_all_fit = true;
    unsigned int count = 0;

    for (const auto& index: blocks)
    {
        const auto block = layer_->getBlockPtr(index);
        if (!block->hasData()) continue;

        const Point origin = block->origin();
        if (std::abs(origin(free_plane_index) - free_plane_val) > block->blockSize()) continue;

        for (size_t linear_index=0; linear_index<num_voxels_per_block; ++linear_index)
        {
            const Point coord = block->calCoordinate(linear_index);
            const TsdfVoxel& voxel = block->getConstVoxel(linear_index);
            if (std::abs(coord(free_plane_index) - free_plane_val) > block->voxelSize()) continue;

            const Scalar distance = voxel.distance_;
            const Scalar weight = voxel.weight_;
            if (count < positions.cols()) positions.col(count) = coord;
            else did_all_fit = false;

            if (count < distances.size()) 
            {
                distances(count) = distance;
                weights(count) = weight;
            }
            else did_all_fit = false;
            
            ++count;
            if (count > max_points) return count;
        }
    }

    if (!did_all_fit)
    {
        throw std::runtime_error(
            std::string("Unable to store ") + std::to_string(count) + " values."
        );
    }
    return count;
}

bool TsdfMap::getWeightAtPosition(const Vector3& position, Scalar& wegiht) const
{
    constexpr bool interpolate = true;
    return getWeightAtPosition(position, wegiht, interpolate);
}

bool TsdfMap::getWeightAtPosition(const Vector3& position, Scalar& wegiht, const bool interpolate) const
{
    return interpolator_.getWeight(position, wegiht, interpolate);
}

}