#include <sdf_generator/integrator/merge_tsdf_integrator.hpp>

namespace sdf_generator
{

MergeTsdfIntegrator::MergeTsdfIntegrator(
    const Config& config, Layer<TsdfVoxel>::Ptr layer
)
: TsdfIntegratorBase(config, layer)
{}

MergeTsdfIntegrator::~MergeTsdfIntegrator() {}

void MergeTsdfIntegrator::integratePointArray(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, const bool freespace_points
)
{
    // Pre-compute a list of unique voxels to end on.
    // Create a hashmap: VOXEL INDEX -> index in original cloud.
    LongIndexHashMapType<AlignedVector<size_t>>::type voxel_map;
    // This is a hash map (same as above) to all the indices that need to be
    // cleared.
    LongIndexHashMapType<AlignedVector<size_t>>::type clear_map;

    std::unique_ptr<ThreadSafeIndex> index_getter(
        ThreadSafeIndexFactory::get(config_.integration_order_mode_, points_c)
    );

    // bundle rays in each voxel with point inside
    bundleRays(
        tf_global2current, points_c, freespace_points, index_getter.get(), 
        voxel_map, clear_map
    );

    // integrate rays for non-clearing voxel (close to the surface)
    integrateRays(
        tf_global2current, points_c, normals_c, colors, 
        config_.enable_anti_grazing_, false, voxel_map, clear_map
    );
}

void MergeTsdfIntegrator::bundleRays(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
    const bool freespace_points, ThreadSafeIndex* index_getter, 
    LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map, 
    LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map
)
{
    size_t point_index;
    while (index_getter->getNextIndex(point_index))
    {
        const Point& point_c = points_c[point_index];
        bool is_clearing;
        if (!isPointValid(point_c, freespace_points, is_clearing)) continue;

        const Point point_g = tf_global2current*point_c;
        GlobalIndex voxel_index = calGridIndex<GlobalIndex>(point_g, voxel_size_inv_);

        if (is_clearing)
        {
            clear_map.emplace(voxel_index, point_index);
        }
        else
        {
            voxel_map.emplace(voxel_index, point_index);
        }
    }
}

void MergeTsdfIntegrator::integrateRays(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
    const Vector3Array& normals_c, const ColorArray& colors, 
    bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map
)
{
    // if only 1 thread just do function call, otherwise spawn threads
    if (config_.integrator_threads_ == 1)
    {
        constexpr size_t thread_index = 0;
        integrateVoxels(
            tf_global2current, points_c, normals_c, colors, enable_anti_grazing, 
            clearing_ray, clear_map, voxel_map, thread_index
        );
    }
    else 
    {
        std::vector<std::thread> integration_threads;
        for (size_t i=0; i<config_.integrator_threads_; ++i)
        {
            integration_threads.emplace_back(
                &MergeTsdfIntegrator::integrateVoxels, this, tf_global2current, 
                points_c, normals_c, colors, enable_anti_grazing, clearing_ray,
                clear_map, voxel_map, i
            );
        }
        for (auto& thread : integration_threads)
        {
            thread.join();
        }
    }

    updateLayerWithStoredBlocks();
}

void MergeTsdfIntegrator::integrateVoxels(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
    const Vector3Array& normals_c, const ColorArray& colors, 
    bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map, 
    size_t thread_index
)
{
    LongIndexHashMapType<AlignedVector<size_t>>::type::const_iterator it;
    size_t map_size;
    if (clearing_ray)
    {
        it = clear_map.begin();
        map_size = clear_map.size();
    }
    else 
    {
        it = voxel_map.begin();
        map_size = voxel_map.size();
    }

    for (size_t i=0; i<map_size; ++i)
    {
        if (((i+thread_index + 1)%config_.integrator_threads_) == 0)
        {
            integrateVoxel(
                tf_global2current, points_c, normals_c, colors, enable_anti_grazing, 
                clearing_ray, *it, voxel_map
            );
        }
        ++it;
    }
}

void MergeTsdfIntegrator::integrateVoxel(
    const TransformMatrix<Scalar>& tf_global2current, const PointArray& points_c, 
        const Vector3Array& normals_c, const ColorArray& colors, 
        bool enable_anti_grazing, bool clearing_ray,
        const std::pair<GlobalIndex, AlignedVector<size_t>>& kv, 
        const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map
)
{
    if (kv.second.empty()) return;

    const Point& origin = tf_global2current.translation();
    Color merged_color;
    Point merged_point_c{0.0, 0.0, 0.0};
    Vector3 merged_normal_c{0.0, 0.0, 0.0};
    Scalar merged_weight = 0.0;

    for (const size_t point_index: kv.second)
    {
        const Point& point_c = points_c[point_index];
        const Vector3& normal_c = normals_c[point_index];
        const Color& color = colors[point_index];

        const Scalar point_weight = getVoxelWeight(point_c);
        if (point_weight < kEpsilon) continue;

        merged_point_c = (merged_point_c*merged_weight + point_c*point_weight) / (merged_weight + point_weight);
        merged_color = Color::blendTwoColors(merged_color, merged_weight, color, point_weight);

        if (config_.normal_available_)
        {
            merged_normal_c = merged_normal_c*merged_weight + normal_c*point_weight;
            if (merged_normal_c.squaredNorm() > kEpsilon*kEpsilon) merged_normal_c.normalize();
        }
        merged_weight += point_weight;

        // only take first point when clearing
        if (clearing_ray) break;
    }

    const Point merged_point_g = tf_global2current*merged_point_c;
    const Vector3 merged_normal_g = tf_global2current.rotation()*merged_normal_c;

    RayCaster ray_caster(
        origin, merged_point_g, clearing_ray, config_.voxel_carving_enabled_, 
        config_.max_ray_length_, voxel_size_inv_, 
        config_.default_truncation_distance_
    );

    GlobalIndex global_voxel_index;
    while (ray_caster.nextRayIndex(global_voxel_index))
    {
        if (enable_anti_grazing)
        {
            // Check if this one is already the block hash map 
            // for this intersection. 
            // Skip this to avoid grazing
            if ((clearing_ray || global_voxel_index != kv.first)
            && voxel_map.find(global_voxel_index) != voxel_map.end())
            {
                continue;
            }
        }

        Block<TsdfVoxel>::Ptr block = nullptr;
        BlockIndex block_index;
        TsdfVoxel* voxel = allocateStorageAndGetVoxelPtr(global_voxel_index, block, block_index);

        updateTsdfVoxel(
            tf_global2current, origin, merged_point_c, merged_point_g, 
            merged_normal_c, merged_normal_g, global_voxel_index, 
            merged_color, merged_weight, *voxel
    );
    }
}

}