#include <sdf_generator/integrator/neighbor_tools.hpp>

namespace sdf_generator
{
// clang-format off
const NeighborhoodLookupTables::Distances NeighborhoodLookupTables::kDistances = [] 
{ 
    const float sqrt_2 = std::sqrt(2);
    const float sqrt_3 = std::sqrt(3);
    Distances distance_matrix;
    distance_matrix <<  1.f, 1.f, 1.f, 1.f, 1.f, 1.f,
                        sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                        sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                        sqrt_2, sqrt_2, sqrt_2, sqrt_2,
                        sqrt_3, sqrt_3, sqrt_3, sqrt_3,
                        sqrt_3, sqrt_3, sqrt_3, sqrt_3;
    return distance_matrix;
}();


const NeighborhoodLookupTables::IndexOffsets NeighborhoodLookupTables::kOffsets = [] 
{
    IndexOffsets directions_matrix;
    directions_matrix << -1,  1,  0,  0,  0,  0, -1, -1,  1,  1,  0,  0,  0,  0, -1,  1, -1,  1, -1, -1, -1, -1,  1,  1,  1,  1, // NOLINT
                            0,  0, -1,  1,  0,  0, -1,  1, -1,  1, -1, -1,  1,  1,  0,  0,  0,  0, -1, -1,  1,  1, -1, -1,  1,  1, // NOLINT
                            0,  0,  0,  0, -1,  1,  0,  0,  0,  0, -1,  1, -1,  1, -1, -1,  1,  1, -1,  1, -1,  1, -1,  1, -1,  1; // NOLINT
    return directions_matrix;
}();

const NeighborhoodLookupTables::LongIndexOffsets NeighborhoodLookupTables::kLongOffsets = [] 
{
    return NeighborhoodLookupTables::kOffsets.cast<LongIndexElement>(); // NOLINT
}();
// clang-format on
}