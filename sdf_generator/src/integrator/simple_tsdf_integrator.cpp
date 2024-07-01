#include <sdf_generator/integrator/simple_tsdf_integrator.hpp>

namespace sdf_generator
{
void SimpleTsdfIntegrator::integratePointArray(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
    const Vector3Array& normals_c, const ColorArray& colors, 
    const bool freespace_points)
{
    std::unique_ptr<ThreadSafeIndex> index_getter(
        ThreadSafeIndexFactory::get(config_.integration_order_mode_, points_c)
    );

    std::vector<std::thread> integration_threads;
    for (size_t i=0; i<config_.integrator_threads_; ++i)
    {
        integration_threads.emplace_back(
            &SimpleTsdfIntegrator::integrateFunction, this, 
            tf_global2current, points_c, normals_c, colors, 
            freespace_points, index_getter.get());
    }

    for (auto& thread: integration_threads) thread.join();

    updateLayerWithStoredBlocks();
    std::cout << "[integratePointArray] updated layer with stored blocks. Num. of block: " << layer_->blockNum() << std::endl;
}

void SimpleTsdfIntegrator::integrateFunction(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c,
    const Vector3Array& normals_c, const ColorArray& colors,
    const bool freespace_points, ThreadSafeIndex* index_getter)
{
    size_t point_index;
    while (index_getter->getNextIndex(point_index))
    {
        const Point& point_c = points_c[point_index];
        const Vector3& normal_c = normals_c[point_index];
        const Color& color = colors[point_index];

        bool is_clearing;
        if (!isPointValid(point_c, freespace_points, is_clearing)) continue;

        const Point& origin = tf_global2current.translation();
        const Point point_g = tf_global2current*point_c;
        const Vector3 normal_g = tf_global2current.rotation()*normal_c;


        // ray caster
        RayCaster ray_caster(
            origin, point_g, is_clearing, 
            config_.voxel_carving_enabled_,
            config_.max_ray_length_, voxel_size_inv_,
            config_.default_truncation_distance_
        );

        Block<TsdfVoxel>::Ptr block = nullptr;
        BlockIndex block_index;
        GlobalIndex global_voxel_index;
        while (ray_caster.nextRayIndex(global_voxel_index))
        {
            TsdfVoxel* voxel = allocateStorageAndGetVoxelPtr(global_voxel_index, block, block_index);
            updateTsdfVoxel(
                tf_global2current, origin, point_c, point_g, 
                normal_c, normal_g, global_voxel_index, color, 0.0f,
                *voxel
            );
        }
    }
}
}