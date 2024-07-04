#include <sdf_generator/integrator/neighbor_tools_24.hpp>

namespace sdf_generator
{
// used only for 24-neighborhood
// clang-format off
const Neighborhood24LookupTables::Distances Neighborhood24LookupTables::kDistances = [] 
{
    const float sqrt_2 = std::sqrt(2);
    Distances distance_matrix;
    distance_matrix <<  1.f, 1.f, 1.f, 1.f, 1.f, 1.f,
                        sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                        sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                        sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                        2.f, 2.f, 2.f, 2.f, 2.f, 2.f;

    return distance_matrix;
}();

const Neighborhood24LookupTables::IndexOffsets Neighborhood24LookupTables::kOffsets = [] 
{
    IndexOffsets directions_matrix;
    directions_matrix << -1,  1,  0,  0,  0,  0, -1,  1,  0,  0, -1,  1, -1,  1,  0,  0,  1, -1, -2,  2,  0,  0,  0,  0,  // NOLINT
                            0,  0, -1,  1,  0,  0, -1,  1, -1,  1,  0,  0,  1, -1, -1,  1,  0,  0,  0,  0, -2,  2,  0, -0,  // NOLINT
                            0,  0,  0,  0, -1,  1,  0,  0, -1,  1, -1,  1,  0,  0,  1, -1, -1,  1,  0,  0,  0,  0, -2,  2;  // NOLINT
    return directions_matrix;
}();

const Neighborhood24LookupTables::LongIndexOffsets Neighborhood24LookupTables::kLongOffsets = [] 
{
    return Neighborhood24LookupTables::kOffsets.cast<LongIndexElement>(); // NOLINT
}();
// clang-format on

}