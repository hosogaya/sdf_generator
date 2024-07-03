#pragma once

#include <memory>
#include <string>
#include <utility>

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/layer.hpp>
#include <sdf_generator/interpolator/interpolator.hpp>

namespace sdf_generator
{
class EsdfMap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<EsdfMap>;

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Scalar esdf_voxel_size_ = 0.2;
        size_t esdf_voxels_per_side = 16;
    };

    explicit EsdfMap(const Config config)
    : esdf_layer_(new Layer<EsdfVoxel>(
        config.esdf_voxel_size_, config.esdf_voxels_per_side
    )), 
    interpolator_(esdf_layer_)
    {
        block_size_ = config.esdf_voxel_size_*config.esdf_voxels_per_side;
    }
    virtual ~EsdfMap() {}

    Layer<EsdfVoxel>::Ptr getEsdfLayerPtr() {return esdf_layer_;}
    const Layer<EsdfVoxel>& getConstEsdfLayer() const {return *esdf_layer_;}

    Scalar blockSize() const {return block_size_;}
    Scalar voxelSize() const {return esdf_layer_->voxelSize();}

    /**
     * Specific accessor functions for esdf maps.
     * Returns true if the point exists in the map AND is observed.
     * These accessors use Vector3d and doubles explicitly rather than
     * Scalar to have a standard, cast-free interface to planning
     * functions.
     */
    bool getDistanceAtPosition(const Eigen::Vector3d& position, double& distance) const;
    bool getDistanceAtPosition(const Eigen::Vector3d& position, double& distance, bool interpolate) const;

    bool getDistanceAndGradientAtPosition(
        const Eigen::Vector3d& position, double& distance, 
        Eigen::Vector3d& gradient) const;
    bool getDistanceAndGradientAtPosition(
        const Eigen::Vector3d& position, double& distance, 
        Eigen::Vector3d& gradient, bool interpolate) const;

    bool isObserved(const Eigen::Vector3d& position) const;

    // EigenDRef is fully dynamic stride type alias for Numpy array slices
    // Use column-major matrices; column-by-column traversal is faster
    // Convenience alias borrowed from pybind11
    using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    template <typename MatrixType>
    using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;

    void batchGetDistanceAtPosition(
        EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions, 
        Eigen::Ref<Eigen::VectorXd> distances, 
        Eigen::Ref<Eigen::VectorXi> observed
    ) const;

    void batchGetDisntaceAndGradientAtPosition(
        EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions, 
        Eigen::Ref<Eigen::VectorXd> distances, 
        EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& gradients, 
        Eigen::Ref<Eigen::VectorXi> observed
    );

    void batchIsObserved(
        EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions, 
        Eigen::Ref<Eigen::VectorXi> observed
    ) const;

    // unsigned int coordPlanesSliceGetCount(
    //     unsigned int free_plane_index, double free_plane_val
    // ) const;

    /**
     * Extract all voxels on a slice plane that is parallel to one of the
     * axis-aligned planes. free_plane_index specifies the free coordinate
     * (zero-based; x, y, z order) free_plane_val specifies the plane intercept
     * coordinate along that axis
     */
    unsigned int coordPlaneSliceGetDistance(
        unsigned int free_plane_index, double free_plane_val,
        EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,  
        Eigen::Ref<Eigen::VectorXd> distances, unsigned int max_points
    ) const;

protected:
    Scalar block_size_;
    Layer<EsdfVoxel>::Ptr esdf_layer_;
    Interpolator<EsdfVoxel> interpolator_;
};
}