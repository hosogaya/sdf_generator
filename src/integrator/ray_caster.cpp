#include <sdf_generator/integrator/ray_caster.hpp>

namespace sdf_generator
{
RayCaster::RayCaster(
    const Point& origin, const Point& point_g,
        const bool is_clearing_ray, const bool voxel_carving_enabled,
        const Scalar max_ray_length, const Scalar voxel_size_inv,
        const Scalar truncation_distance, const bool cast_from_origin = true)
{
    const Vector3 unit_ray = (point_g - origin).normalized();

    Point ray_start, ray_end;
    if (is_clearing_ray)
    {
        Scalar ray_length = (point_g - origin).norm();
        ray_length = std::min(
            std::max(
                ray_length - truncation_distance, (0.0f)
            ), 
            max_ray_length
        );

        ray_end = origin + unit_ray*ray_length;
        ray_start = voxel_carving_enabled ? origin : ray_end;
    }
    else 
    {
        ray_end = point_g + unit_ray*truncation_distance;
        ray_start = voxel_carving_enabled ? origin : (point_g - unit_ray*truncation_distance);
    }

    const Point start_scaled = ray_start*voxel_size_inv;
    const Point end_scaled = ray_end * voxel_size_inv;

    if (cast_from_origin) {
        // from start to end
        setupRayCaster(start_scaled, end_scaled);
    } else {
        setupRayCaster(end_scaled, start_scaled);
    }
}

RayCaster::RayCaster(const Point& start_scaled, const Point& end_scaled) 
{
    setupRayCaster(start_scaled, end_scaled);
}

bool RayCaster::nextRayIndex(GlobalIndex& ray_index)
{
    if (current_step_++ > ray_length_in_steps_) return false;

    ray_index = curr_index_;

    int t_min_index;
    t_to_next_boundary_.minCoeff(&t_min_index);
    curr_index_[t_min_index] += ray_step_signs_[t_min_index];
    t_to_next_boundary_[t_min_index] += t_step_size_[t_min_index];

    return true;
}

void RayCaster::setupRayCaster(const Point& start_scaled, const Point& end_scaled)
{
    if (std::isnan(start_scaled.x()) || std::isnan(start_scaled.y())
        || std::isnan(start_scaled.z()) || std::isnan(end_scaled.x())
        || std::isnan(end_scaled.y()) || std::isnan(end_scaled.z())
    )
    {
        ray_length_in_steps_ = 0;
        return;
    } 

    curr_index_ = calGridIndex<GlobalIndex>(start_scaled);
    const GlobalIndex end_index = calGridIndex<GlobalIndex>(end_scaled);
    const GlobalIndex diff_index = end_index - curr_index_;

    current_step_ = 0;

    ray_length_in_steps_ = std::abs(diff_index.x()) + std::abs(diff_index.y()) + std::abs(diff_index.z());

    const Vector3 ray_scaled = end_scaled - start_scaled;

    ray_step_signs_ = AnyIndex(
        signum(ray_scaled.x()), signum(ray_scaled.y()), signum(ray_scaled.z())
    );

    const AnyIndex corrected_step(
        std::max(0, ray_step_signs_.x()), 
        std::max(0, ray_step_signs_.y()), 
        std::max(0, ray_step_signs_.z())
    );

    const Point start_scaled_shifted = start_scaled - curr_index_.cast<Scalar>();

    Vector3 distance_to_boundaries(corrected_step.cast<Scalar>() - start_scaled_shifted);


    t_to_next_boundary_.x() = ray_scaled.x() == 0.0f ? 2.0f : distance_to_boundaries.x()/ray_scaled.x();
    t_to_next_boundary_.y() = ray_scaled.y() == 0.0f ? 2.0f : distance_to_boundaries.y()/ray_scaled.y();
    t_to_next_boundary_.z() = ray_scaled.z() == 0.0f ? 2.0f : distance_to_boundaries.z()/ray_scaled.z();

    t_step_size_.x() = ray_scaled.x() == 0.0f ? 2.0f : ray_step_signs_.x()/ray_scaled.x();
    t_step_size_.y() = ray_scaled.y() == 0.0f ? 2.0f : ray_step_signs_.y()/ray_scaled.y();
    t_step_size_.z() = ray_scaled.z() == 0.0f ? 2.0f : ray_step_signs_.z()/ray_scaled.z();
}

}