#pragma once

#include <sdf_generator/core/layer.hpp>


namespace sdf_generator
{
template <typename VoxelType>
class Interpolator 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Interpolator>;

    explicit Interpolator(const typename Layer<VoxelType>::Ptr layer)
    : layer_(layer) {}
    virtual ~Interpolator() {}

    bool getDistance(const Point& pos, Scalar& distance, bool interpolate = false) const;
    bool getWeight(const Point& pos, Scalar& weight, bool interpolate = false) const;
    bool getVoxel(const Point& pos,  VoxelType& voxel, bool interpolate = false) const;
    bool getGradient(const Point& pos, Vector3& grad, bool interpolate = false) const;

    // bool getAdaptiveDistanceAndGradient(const Point& point, Scalar& distance, Vector3& grad) const;
    // bool getNearestDistanceAndWeight(const Point& point, Scalar& distance, Scalar& weight) const;
    bool setIndexes(const Point& pos, BlockIndex& block_index, InterpIndexes& voxel_indexes) const;
    bool getVoxelsAndQVector(const Point& pos, const VoxelType** voxels, InterpVector& q_vector) const;
    bool getVoxelsAndQVector(const BlockIndex& block_index, const InterpIndexes& voxel_indexes,
                            const Point& pos, const VoxelType** voxels, InterpVector& q_vector) const; 
private:
    /**
     * Q vector from http://spie.org/samples/PM159.pdf
     * Relates the interpolation distance of any arbitrary point inside a voxel
     * to the values of the voxel corners.
     */
    void getQVector(const Point& voxel_pos, const Point& pos, const Scalar voxel_size_inv, InterpVector* q_vector) const;
    template <typename Getter_t>
    
    static Scalar interpMember(const InterpVector& q_vector, const VoxelType** voxels, Getter_t (*getter)(const VoxelType&));
    static VoxelType interpVoxel(const InterpVector& q_vector, const VoxelType** voxels);

    static Scalar getVoxelDistance(const VoxelType& voxel);
    static Scalar getVoxelWeight(const VoxelType& voxel);
    static Color::Value getRed(const VoxelType& voxel);
    static Color::Value getGreen(const VoxelType& voxel);
    static Color::Value getBlue(const VoxelType& voxel);
    static Color::Value getAlpha(const VoxelType& voxel);


    bool getInterpDistance(const Point& pos, Scalar& distance) const;
    bool getInterpWeight(const Point& pos, Scalar& weight) const;
    bool getInterpVoxel(const Point& pos, VoxelType& voxel) const;

    bool getNearestDistance(const Point& pos, Scalar& distance) const;
    bool getNearestWeight(const Point& pos, Scalar& weight) const;
    bool getNearestVoxel(const Point& pos, VoxelType& voxel) const;

    const typename Layer<VoxelType>::Ptr layer_;
};
}

#include <sdf_generator/interpolator/interpolator_impl.hpp>