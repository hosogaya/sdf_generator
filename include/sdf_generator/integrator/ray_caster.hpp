#pragma once

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/util.hpp>

namespace sdf_generator
{
class RayCaster
{
public:
    RayCaster(
        const Point& origin, const Point& point_g,
        const bool is_clearing_ray, const bool voxel_carving_enabled_,
        const Scalar max_ray_length, const Scalar voxel_size_inv,
        const Scalar truncation_distance, const bool cast_from_origin = true);

    RayCaster(const Point& start_scaled, const Point& end_scaled);

    bool nextRayIndex(GlobalIndex& ray_index);

private:
    void setupRayCaster(const Point& start_scaled, const Point& end_scaled);

    Vector3 t_to_next_boundary_;
    GlobalIndex curr_index_;
    AnyIndex ray_step_signs_;
    Vector3 t_step_size_;

    uint ray_length_in_steps_;
    uint current_step_;
};
}