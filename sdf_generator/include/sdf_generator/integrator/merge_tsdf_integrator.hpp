#pragma once

/**
 * Uses ray bundling to improve integration speed, points which lie in the same
 * voxel are "merged" into a single point. Raycasting and updating then proceeds
 * as normal. Fast for large voxels, with minimal loss of information.
 */

#include <sdf_generator/core/tsdf_map.hpp>
#include <sdf_generator/integrator/tsdf_integrator.hpp>
#include <sdf_generator/integrator/thread_safe_index.hpp>
#include <sdf_generator/integrator/ray_caster.hpp>

namespace sdf_generator
{
class MergeTsdfIntegrator: public TsdfIntegratorBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MergeTsdfIntegrator(
        const Config& config, Layer<TsdfVoxel>::Ptr layer);

    ~MergeTsdfIntegrator();

    void integratePointArray(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, const bool freespace_points = false
    ) override;

protected:
    void bundleRays(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const bool freespace_points, ThreadSafeIndex* index_getter, 
        LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map, 
        LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map
    );

    void integrateVoxel(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, 
        bool enable_anti_grazing, bool clearing_ray,
        const std::pair<GlobalIndex, AlignedVector<size_t>>& kv, 
        const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map
    );

    void integrateVoxels(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, 
        bool enable_anti_grazing, bool clearing_ray,
        const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
        const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map, 
        size_t thread_index
    );

    void integrateRays(
        const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, 
        bool enable_anti_grazing, bool clearing_ray,
        const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
        const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map
    );
};
}