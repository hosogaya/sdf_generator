#pragma once

#include <sdf_generator/interpolator/interpolator.hpp>
#include <sdf_generator/interpolator/util.hpp>

namespace sdf_generator
{
template <typename VoxelType>
Interpolator<VoxelType>::Interpolator(const typename Layer<VoxelType>::Ptr layer)
: layer_(layer)
{}

template <typename VoxelType>
bool Interpolator<VoxelType>::getDistance(const Point& pos, Scalar& distance, bool interpolate) const
{
    if (interpolate)
    {
        return getInterpDistance(pos, distance);
    }
    else
    {

    }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpDistance(const Point& pos, Scalar& distance) const
{
    const VoxelType* voxels[8];
    InterpVector q_vector;
    if (!getVoxelsAndQVector(pos, voxels, q_vector))
    {
        return false;
    }
    else
    {
        distance = interpMember(q_vector, voxels, &getVoxelSdf);
    }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestDistance(const Point& pos, Scalar& distance) const
{

}



template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxelsAndQVector(
    const BlockIndex& block_index, const InterpIndexes& voxel_indexes,
    const Point& pos, const VoxelType** voxels, InterpVector& q_vector) const
{
    for (int i=0; i<voxel_indexes.cols(); ++i)
    {
        typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockPtrByIndex(block_index);
        if (block_ptr == nullptr) return false;

        VoxelIndex voxel_index = voxel_indexes.col(i);
        // if voxel index is too large get neighboring block and update index
        if ((voxel_index.array() >= block_ptr->voxelsPerSide()).any())
        {
            BlockIndex new_block_index = block_index;
            for (int j=0; j<block_index.rows(); ++j)
            {
                if (voxel_index(j) > static_cast<IndexElement>(block_ptr->voxelsPerSide()))
                {
                    new_block_index(j)++;
                    voxel_index(j) -= block_ptr->voxelsPerSide();
                }
            }
            block_ptr = layer_->getBlockPtrByIndex(new_block_index);
            if (block_ptr == nullptr) return false;
        }

        // use bottom left corner voxel to compute weights vector
        if (i == 0) getQVector(block_ptr->voxelIndex2Coordinate(voxel_index), pos, block_ptr->voxelSizeInv(), q_vector);
    
        const VoxelType& voxel = block_ptr->getVoxel(voxel_index);

        voxels[i] = & voxel;
        if (isObservedVoxel(voxel))
        {
            return false;
        }
    }
    return true;
}

template <typename VoxelType>
void Interpolator<VoxelType>::getQVector(const Point& voxel_pos, const Point& pos, const Scalar voxel_size_inv, InterpVector* q_vector) const
{
    const Point voxel_offset = (pos - voxel_pos)*voxel_size_inv;

    // FROM PAPER (http://spie.org/samples/PM159.pdf)
    // clang-format off
    *q_vector <<
        1,
        voxel_offset[0],
        voxel_offset[1],
        voxel_offset[2],
        voxel_offset[0] * voxel_offset[1],
        voxel_offset[1] * voxel_offset[2],
        voxel_offset[2] * voxel_offset[0],
        voxel_offset[0] * voxel_offset[1] * voxel_offset[2];
    // clang-format on
}

template <typename VoxelType>
template <typename Getter_t>
inline Scalar Interpolator<VoxelType>::interpMember(const InterpVector& q_vector, const VoxelType** voxels, Getter_t (*getter)(const VoxelType&))
{
    InterpVector data;
    for (int i=0; i<data.size(); +i)
    {
        data[i] = static_cast<Scalar>((*getter)(*voxels[i]));
    }

    // FROM PAPER (http://spie.org/samples/PM159.pdf)
    // clang-format off
    static const InterpTable interp_table =
      (InterpTable() <<
        1,  0,  0,  0,  0,  0,  0,  0,
       -1,  0,  0,  0,  1,  0,  0,  0,
       -1,  0,  1,  0,  0,  0,  0,  0,
       -1,  1,  0,  0,  0,  0,  0,  0,
        1,  0, -1,  0, -1,  0,  1,  0,
        1, -1, -1,  1,  0,  0,  0,  0,
        1, -1,  0,  0, -1,  1,  0,  0,
       -1,  1,  1, -1,  1, -1, -1,  1
       ).finished();

    // clang-format on
    return q_vector * (interp_table * data.transpose());
}

}