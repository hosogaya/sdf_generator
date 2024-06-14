#pragma once

#include <sdf_generator/integrator/tsdf_integrator.hpp>
#include <sdf_generator/integrator/thread_safe_index.hpp>
#include <sdf_generator/integrator/ray_caster.hpp>

namespace sdf_generator
{
/**
 * Basic TSDF integrator. Every point is raycast through all the voxels, which
 * are updated individually. An exact but very slow approach.
 */
class SimpleTsdfIntegrator : public TsdfIntegratorBase 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SimpleTsdfIntegrator(const Config& config, Layer<TsdfVoxel>::Ptr layer)
        : TsdfIntegratorBase(config, layer) {}

    void integratePointArray(
        const TransformMatrix<Scalar>& tf_global2camer, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, 
        const bool freespace_points = false) override;

    void integrateFunction(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c,
        const Vector3Array& normals_c, const ColorArray& colors,
        const bool freespace_points, ThreadSafeIndex* index_getter);
};
}