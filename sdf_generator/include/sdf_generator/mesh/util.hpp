#pragma once

#include <vector>
#include <sdf_generator/core/block.hpp>
#include <sdf_generator/core/hash.hpp>
#include <sdf_generator/mesh/mesh.hpp>

namespace sdf_generator
{
inline bool createConnectedMesh
(
    const AlignedVector<Mesh::ConstPtr>& meshes, 
    Mesh& connected_mesh, 
    const Scalar approximate_vertex_proximity_threshold =1e-10
)
{
    LongIndexHashMapType<size_t>::type uniques;

    const Scalar threshold_inv = 1.0/approximate_vertex_proximity_threshold;

    size_t new_vertex_index = 0;
    for (const Mesh::ConstPtr& mesh: meshes)
    {
        if (mesh->vertices_.empty()) continue;

        // Make sure there are 3 distinct vertices for every triangle before
        // merging.
        if (mesh->vertices_.size() != mesh->indices_.size()) return false;
        if (mesh->vertices_.size()%3 != 0) return false;
        // if (mesh->indices_.size()%3 != 0) return false;

        // Stores the mapping from old vertex index to the new one in the combined
        // mesh. This is later used to adapt the triangles to the new, global
        // indexing of the combined mesh.
        std::vector<size_t> old_to_new_indices(mesh->vertices_.size());

        size_t new_num_vertices_from_this_block = 0u;
        for (size_t old_vertex_index =0u; old_vertex_index < mesh->vertices_.size(); ++old_vertex_index)
        {
            // We scale the vertices by the inverse of the merging tolerance and
            // then compute a discretized grid index in that scale.
            // This exhibits the behaviour of merging two vertices that are
            // closer than the threshold.
            const Point& vertex = mesh->vertices_[old_vertex_index];
            const Vector3 scaled_vector(vertex*threshold_inv);

            const LongIndex vertex_3D_index{
                std::round(scaled_vector.x()), 
                std::round(scaled_vector.y()), 
                std::round(scaled_vector.z()) 
            };

            // If the current vertex falls into the same grid cell as a previous
            // vertex, we merge them. This is done by assigning the new vertex to
            // the same vertex index as the first vertex that fell into that cell.

            LongIndexHashMapType<size_t>::type::const_iterator it =uniques.find(vertex_3D_index);

            const bool vertex_is_unique_so_far = (it == uniques.end());
            if (vertex_is_unique_so_far)
            {
                // Copy vertex and associated data to combined mesh.
                connected_mesh.vertices_.push_back(vertex);
            
                if (mesh->hasColors())
                    connected_mesh.colors_.push_back(mesh->colors_[old_vertex_index]);

                if (mesh->hasNormals())
                    connected_mesh.normals_.push_back(mesh->normals_[old_vertex_index]);
                
                // Store the new vertex index in the unique-vertex-map to be able to
                // retrieve this index later if we encounter vertices that are
                // supposed to be merged with this one.
                uniques.emplace(vertex_3D_index, new_vertex_index);
                
                // Also store a mapping from old index to new index for this mesh
                // block to later adapt the triangle indexing.
                old_to_new_indices[old_vertex_index] = new_vertex_index;

                ++new_vertex_index;
                ++new_num_vertices_from_this_block;
            }
            else 
            {
                // If this vertex is not unique, we map it's vertex index to the new
                // vertex index.
                old_to_new_indices[old_vertex_index] = it->second;

                // Add all normals (this will average them once they are renormalized
                // later)
                connected_mesh.normals_[it->second] += mesh->normals_[old_vertex_index];
            }
        }

        // Make sure we have a mapping for every old vertex index.
        if (old_to_new_indices.size() != mesh->vertices_.size()) return false;;

        // Renormalize normals
        for (Point& normal : connected_mesh.normals_) {
            Scalar length = normal.norm();
            if (length > kEpsilon) {
                normal /= length;
            } else {
                normal = Point(0.0f, 0.0f, 1.0f);
            }
        }

        // Append triangles and adjust their indices if necessary.
        // We discard triangles where all old vertices were mapped to the same
        // vertex.
        size_t new_num_triangle_from_this_block = 0u;
        for (size_t triangle_index = 0u; triangle_index < mesh->indices_.size(); triangle_index += 3u) 
        {
            if (triangle_index+2 >= mesh->indices_.size()) return false;

            // Retrieve old vertex indices.
            size_t vertex_0 = mesh->indices_[triangle_index];
            size_t vertex_1 = mesh->indices_[triangle_index + 1u];
            size_t vertex_2 = mesh->indices_[triangle_index + 2u];

            // Make sure the old indices were valid before remapping.
            if (vertex_0 >= old_to_new_indices.size()) return false;
            if (vertex_1 >= old_to_new_indices.size()) return false;
            if (vertex_2 >= old_to_new_indices.size()) return false;

            // Apply vertex index mapping.
            vertex_0 = old_to_new_indices[vertex_0];
            vertex_1 = old_to_new_indices[vertex_1];
            vertex_2 = old_to_new_indices[vertex_2];

            // Make sure the new indices are valid after remapping.
            if (vertex_0 >= new_vertex_index) return false;
            if (vertex_1 >= new_vertex_index) return false;
            if (vertex_2 >= new_vertex_index) return false;

            // Get rid of triangles where all two or three vertices have been
            // merged.
            const bool two_or_three_vertex_indices_are_the_same =
                (vertex_0 == vertex_1) || (vertex_1 == vertex_2) ||
                (vertex_0 == vertex_2);

            if (!two_or_three_vertex_indices_are_the_same) {
                connected_mesh.indices_.push_back(vertex_0);
                connected_mesh.indices_.push_back(vertex_1);
                connected_mesh.indices_.push_back(vertex_2);
                ++new_num_triangle_from_this_block;
            }
        }
    }

    // Verify combined mesh.
    if (connected_mesh.hasColors() && (connected_mesh.vertices_.size() != connected_mesh.colors_.size())) return false;
    if (connected_mesh.hasNormals() && (connected_mesh.vertices_.size(), connected_mesh.normals_.size())) return false;

    return true;
}

inline bool createConnectedMesh
(
    const Mesh& mesh, Mesh& connected_mesh,
    const Scalar approximate_vertex_proximity_threshold = 1e-10
)
{
    AlignedVector<Mesh::ConstPtr> meshes;
    meshes.push_back(Mesh::ConstPtr(&mesh, [](Mesh const*){}));
    createConnectedMesh(meshes, connected_mesh, approximate_vertex_proximity_threshold);
}

template <typename VoxelType>
bool getSdfIfValid(
    const VoxelType& voxel, const Scalar min_weight, Scalar& sdf);

/*******************************
 * probability
 ********************************/
// template <>
// inline bool getSdfIfValid(
//     const TsdfVoxel& voxel, const Scalar min_weight, Scalar& sdf)
// {
//     if (voxel.probability_ <= min_weight) 
//     {
//         // std::cout << "[getSdfIfValid] the weight " << voxel.probability_ << " is smaller than " << min_weight << std::endl;
//         return false;
//     }
//     sdf = voxel.distance_;
//     return true;
// }

/*******************************
 * weight
 ********************************/
template <>
inline bool getSdfIfValid(
    const TsdfVoxel& voxel, const Scalar min_weight, Scalar& sdf)
{
    if (voxel.weight_ <= min_weight) 
    {
        // std::cout << "[getSdfIfValid] the weight " << voxel.probability_ << " is smaller than " << min_weight << std::endl;
        return false;
    }
    sdf = voxel.distance_;
    return true;
}

/*******************************
 * distance
 ********************************/
// template <>
// inline bool getSdfIfValid(
//     const TsdfVoxel& voxel, const Scalar truncate_distance, Scalar& sdf)
// {
//     if (voxel.distance_ > truncate_distance) 
//     {
//         // std::cout << "[getSdfIfValid] the weight " << voxel.probability_ << " is smaller than " << min_weight << std::endl;
//         return false;
//     }
//     sdf = voxel.distance_;
//     return true;
// }

template <>
inline bool getSdfIfValid(
    const EsdfVoxel& voxel, const Scalar min_weight, Scalar& sdf)
{
  if (!voxel.observed_) return false;
  sdf = voxel.distance_;
  return true;
}

template <typename VoxelType>
bool getColorIfValid(
    const VoxelType& voxel, const Scalar min_weight, Color& color);


/*******************************
 * probability
 ********************************/
// template <>
// inline bool getColorIfValid(
//     const TsdfVoxel& voxel, const Scalar min_weight, Color& color)
// {
//     if (voxel.probability_ <= min_weight) {
//         return false;
//     }
//     color = voxel.color_;
//     // color.a_ = voxel.probability_*std::numeric_limits<Color::Value>::max();
//     return true;
// }

/*******************************
 * weight
 ********************************/
template <>
inline bool getColorIfValid(
    const TsdfVoxel& voxel, const Scalar min_weight, Color& color)
{
    if (voxel.weight_ <= min_weight) {
        return false;
    }
    color = voxel.color_;
    // color.a_ = voxel.probability_*std::numeric_limits<Color::Value>::max();
    return true;
}

/*******************************
 * distance
 ********************************/
// template <>
// inline bool getColorIfValid(
//     const TsdfVoxel& voxel, const Scalar truncate_distance, Color& color)
// {
//     if (voxel.distance_ > truncate_distance) {
//         return false;
//     }
//     color = voxel.color_;
//     // color.a_ = voxel.probability_*std::numeric_limits<Color::Value>::max();
//     return true;
// }


template <>
inline bool getColorIfValid(
    const EsdfVoxel& voxel, const Scalar min_weight, Color& color)
{
    if (!voxel.observed_) {
        return false;
    }
    color = Color(255u, 255u, 255u);
    return true;
}

}