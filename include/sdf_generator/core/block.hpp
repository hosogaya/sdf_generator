#pragma once

#include <sdf_generator/core/voxel.hpp>
#include <sdf_generator/core/util.hpp>

namespace sdf_generator
{
template <typename VoxelType>
class Block
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Block<VoxelType>>;
    using ConstPtr = std::shared_ptr<const Block<VoxelType>>;

    Block(size_t voxels_per_side, Scalar voxel_size, const Point& origin)
    : voxel_size_(voxel_size), voxels_per_side_(voxels_per_side), origin_(origin)
    {
        num_voxels_ = std::pow(voxels_per_side_, 3.0f);
        voxel_size_inv_ = 1.0/voxel_size_;
        block_size_ = voxel_size_*voxels_per_side_;
        block_size_inv_ = 1.0/block_size_;
        voxels_.reset(new VoxelType[num_voxels_]);
    }
    ~Block() {}

    size_t voxelIndex2LinearIndex(const VoxelIndex& index) const
    {
        DCHECK(index.x() >=0 && index.x() <voxels_per_side_);
        DCHECK(index.y() >=0 && index.y() <voxels_per_side_);
        DCHECK(index.z() >=0 && index.z() <voxels_per_side_);

        return static_cast<size_t>(index.x() + index.y()*voxels_per_side_ + index.z()*voxels_per_side_*voxels_per_side_);
    }

    size_t coordinate2LinearIndex(const Point& coords) const
    {
        return voxelIndex2LinearIndex(coordinate2VoxelIndex(coords));
    }

    VoxelIndex coordinate2VoxelIndex(const Point& coords) const
    {
        VoxelIndex index = point2voxelIndex<VoxelIndex>(coords - origin_, voxel_size_inv_);
        Scalar max_value = voxels_per_side_ - 1;
        return VoxelIndex(
            std::max(std::min(index.x(), max_value), 0), 
            std::max(std::min(index.y(), max_value), 0), 
            std::max(std::min(index.z(), max_value), 0), 
        );
    }

    // https://cpprefjp.github.io/reference/cstdlib/div.html 
    // std::div_t has quotient(商) and remainder(剰余)
    inline VoxelIndex linearIndex2VoxelIndex(const size_t index) const
    {
        int rem = index;
        VoxelIndex result;
        std::div_t div_temp = std::div(rem, voxels_per_side_ * voxels_per_side_);
        rem = div_temp.rem;
        result.z() = div_temp.quot;
        div_temp = std::div(rem, voxels_per_side_);
        result.y() = div_temp.quot;
        result.x() = div_temp.rem;
        return result;
    }

    Point voxelIndex2Coordinate(const VoxelIndex& index) const
    {
        return origin_ + getCenterPointOfVoxel(index, voxel_size_);
    }

    inline Point linearIndex2Coordinate(const size_t index) const
    {
        return voxelIndex2Coordinate(linearIndex2VoxelIndex(index));
    }

    inline VoxelType& getVoxel(size_t index) 
    {
        return voxels_[index];
    }
    
    inline VoxelTyep& getVoxel(const VoxelIndex& index) 
    {
        return voxels_[linearIndex2VoxelIndex(index)];
    }

    inline VoxelType& getVoxel(const Point& coords)
    {
        return voxels_[coordinate2LinearIndex(coords)];
    }


    size_t voxelsPerSide() const 
    {
        return voxels_per_side_;
    }

    Scalar voxelSize() const 
    {
        return voxel_size_;
    }
    Scalar voxelSizeInv() const 
    {
        return voxel_size_inv_;
    }
    size_t numVoxels() const
    {
        return num_voxels_;
    }
    Point origin() const 
    {
        return origin_;
    }

    void setOrigin(const Point& new_origin)
    {
        origin_ = new_origin;
    }
    Scalar blockSize() const 
    {
        return block_size_;
    }


protected:
    std::unique_ptr<VoxelType[]> voxels_;
    size_t num_voxels_;
    bool has_data_;

private:
    const size_t voxels_per_side_;
    const Scalar voxel_size_;
    Point origin_;

    Scalar voxel_size_inv_;
    Scalar block_size_;
    Scalar block_size_inv_;
};
}