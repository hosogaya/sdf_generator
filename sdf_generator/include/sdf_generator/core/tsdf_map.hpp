#pragma once

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/layer.hpp>
#include <sdf_generator/interpolator/interpolator.hpp>

namespace sdf_generator
{
class TsdfMap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<TsdfMap>;

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        Scalar tsdf_voxel_size_ = 0.2;
        size_t tsdf_voxels_per_side_ = 16u;

    };

    explicit TsdfMap(const Config& config)
    : layer_(new Layer<TsdfVoxel>(config.tsdf_voxel_size_, config.tsdf_voxels_per_side_)), 
    interpolator_(layer_)
    {
        block_size_ = config.tsdf_voxel_size_*config.tsdf_voxels_per_side_;
    }

    explicit TsdfMap(const Layer<TsdfVoxel>& layer)
    : TsdfMap(aligned_shared<Layer<TsdfVoxel>>(layer)) {}

    explicit TsdfMap(const Layer<TsdfVoxel>::Ptr layer)
    : layer_(layer), interpolator_(layer)
    {
        if (!layer) 
        {
            throw std::runtime_error(
                std::string("Null Layer<TsdfVoxel>::Ptr in TsdfMap constructor")
            );
        }
        block_size_ = layer_->blockSize();
    }

    virtual ~TsdfMap() {}

    Layer<TsdfVoxel>::Ptr getTsdfLayerPtr() {return layer_;}
    const Layer<TsdfVoxel>::Ptr getTsdfLayerConstPtr() {return layer_;}
    
    Scalar blockSize() const {return block_size_;}
    Scalar voxelSize() const {return layer_->voxelSize();}

    /* NOTE(mereweth@jpl.nasa.gov)
    * EigenDRef is fully dynamic stride type alias for Numpy array slices
    * Use column-major matrices; column-by-column traversal is faster
    * Convenience alias borrowed from pybind11
    */
    using EigenDstride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    template <typename MatrixType>
    using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDstride>;

    /**
     *  Extract all voxels on a slice plane that is parallel to one of the
     * axis-aligned planes. free_plane_index specifies the free coordinate
     * (zero-based; x, y, z order) free_plane_val specifies the plane intercept
     * coordinate along that axis
     */
    unsigned int extractDistanceAndWeightOfVoxelsInSlicePlane(
        unsigned int free_plane_index, Scalar free_plane_val, 
        EigenDRef<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>>& positions,
        Eigen::Ref<Eigen::VectorX<Scalar>> distances,
        Eigen::Ref<Eigen::VectorX<Scalar>> weights, unsigned int max_points) const;

    bool getWeightAtPosition(const Point& position, Scalar& weight) const;
    bool getWeightAtPosition(const Point& position, Scalar& weight, const bool interpolate) const;

protected:
    Scalar block_size_;
    Layer<TsdfVoxel>::Ptr layer_;
    Interpolator<TsdfVoxel> interpolator_;
};
}