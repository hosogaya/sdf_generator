#pragma once

#include <sdf_generator/mesh/type.hpp>
#include <sdf_generator/mesh/mesh.hpp>
#include <sdf_generator/mesh/util.hpp>
#include <sdf_generator/core/hash.hpp>
#include <sdf_generator/core/util.hpp>

namespace sdf_generator
{
class MeshLayer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<MeshLayer>;
    using ConstPtr = std::shared_ptr<const MeshLayer>;
    using MeshMap = typename AnyIndexHashMapType<Mesh::Ptr>::type;

    explicit MeshLayer(Scalar block_size)
    : block_size_(block_size), block_size_inv_(1.0/block_size)
    {}

    virtual ~MeshLayer() {}

    inline const Mesh& getConstMesh(const BlockIndex& index) const
    {
        typename MeshMap::const_iterator it = mesh_map_.find(index);
        
        assert(it != mesh_map_.end());
        return *it->second;
    }

    inline Mesh& getMesh(const BlockIndex& index)
    {
        typename MeshMap::iterator it = mesh_map_.find(index);
        
        assert(it != mesh_map_.end());
        return *it->second;
    }

    inline typename Mesh::ConstPtr getMeshConstPtr(const BlockIndex& index) const
    {
        typename MeshMap::const_iterator it = mesh_map_.find(index);
        if (it != mesh_map_.end()) return it->second;

        return typename Mesh::ConstPtr();
    }
    inline typename Mesh::ConstPtr getMeshConstPtr(const Point& coords) const
    {
        return getMeshConstPtr(calGridIndex<BlockIndex>(coords, block_size_inv_));
    }

    inline typename Mesh::Ptr getMeshPtr(const BlockIndex& index)
    {
        typename MeshMap::iterator it = mesh_map_.find(index);
        if (it != mesh_map_.end()) return it->second;

        return typename Mesh::Ptr();
    }

    inline typename Mesh::Ptr getMeshPtr(const Point& coords) 
    {
        return getMeshPtr(calGridIndex<BlockIndex>(coords, block_size_inv_));
    }

    typename Mesh::Ptr allocateNewBlock(const BlockIndex& index)
    {
        auto insert_status = mesh_map_.insert(
            std::make_pair(index, std::shared_ptr<Mesh>(
                new Mesh(block_size_, index.cast<Scalar>()*block_size_)
            ))
        );

        return insert_status.first->second;
    }

    inline typename Mesh::Ptr allocateMeshPtr(const BlockIndex& index)
    {
        typename MeshMap::const_iterator it = mesh_map_.find(index);
        if (it != mesh_map_.end()) return it->second;

        return allocateNewBlock(index);
    }

    inline typename Mesh::Ptr allocateMeshPtr(const Point& coords)
    {
        return allocateMeshPtr(calGridIndex<BlockIndex>(coords, block_size_inv_));
    }

    void removeMesh(const BlockIndex& index) {mesh_map_.erase(index);}

    void removeMesh(const Point& coords) {
        mesh_map_.erase(calGridIndex<BlockIndex>(coords, block_size_inv_));
    }

    void clearDistanceMesh(const Point& center, const double max_distance)
    {
        // we clear the mesh, but do not delete it from the map as the empty mesh
        // must be sent to rviz so it is also cleared there
        for (std::pair<const BlockIndex, typename Mesh::Ptr>& kv : mesh_map_) {
            if ((kv.second->origin_ - center).squaredNorm() >
                max_distance * max_distance) {
                kv.second->clear();
                kv.second->update_ = true;
            }
        }
    }

    void getAllocatedMeshes(BlockIndexList* meshes) const 
    {
        meshes->clear();
        meshes->reserve(mesh_map_.size());
        for (const std::pair<const BlockIndex, typename Mesh::Ptr>& kv: mesh_map_)
        {
            meshes->emplace_back(kv.first);
        }
    }

    void getAllUpdatedMeshes(BlockIndexList* meshes) const
    {
        meshes->clear();
        meshes->reserve(mesh_map_.size());
        for (const std::pair<const BlockIndex, typename Mesh::Ptr>& kv: mesh_map_)
        {
            if (kv.second->update_)
                meshes->emplace_back(kv.first);
        }
    }

    void getMesh(Mesh& combined_mesh) const
    {
        BlockIndexList mesh_indices;
        getAllocatedMeshes(&mesh_indices);

        // Check if color, normals and indices are enabled for the first non-empty
        // mesh. If they are, they need to be enabled for all other ones as well.
        bool has_colors = false;
        bool has_normals = false;
        bool has_indices = false;
        if (!mesh_indices.empty())
        {
            for (const BlockIndex& block_index: mesh_indices)
            {
                Mesh::ConstPtr mesh = getMeshConstPtr(block_index);
                if (!mesh->vertices_.empty())
                {
                    has_colors = mesh->hasColors();
                    has_normals = mesh->hasNormals();
                    has_indices = mesh->hasTriangle();
                    break;
                }
            }
        }
        // Loop again over all meshes to figure out how big the mesh needs to be.
        size_t mesh_size = 0;
        for (const BlockIndex& block_index : mesh_indices) {
            Mesh::ConstPtr mesh = getMeshConstPtr(block_index);
            mesh_size += mesh->vertices_.size();
        }

        combined_mesh.reserve(mesh_size, has_normals, has_colors, has_indices);

        size_t new_index = 0;
        for (const BlockIndex& block_index: mesh_indices)
        {
            Mesh::ConstPtr mesh = getMeshConstPtr(block_index);

            // Copy the mesh content into the combined mesh. This is done in triplets
            // for readability only, as one loop iteration will then copy one
            // triangle.
            for (size_t i=0; i<mesh->vertices_.size(); i+=3, new_index+=3)
            {
                combined_mesh.vertices_.push_back(mesh->vertices_[i]);
                combined_mesh.vertices_.push_back(mesh->vertices_[i+1]);
                combined_mesh.vertices_.push_back(mesh->vertices_[i+2]);

                if (has_colors)
                {
                    combined_mesh.colors_.push_back(mesh->colors_[i]);
                    combined_mesh.colors_.push_back(mesh->colors_[i+1]);
                    combined_mesh.colors_.push_back(mesh->colors_[i+2]);
                }
                if (has_normals) {
                    combined_mesh.normals_.push_back(mesh->normals_[i]);
                    combined_mesh.normals_.push_back(mesh->normals_[i+1]);
                    combined_mesh.normals_.push_back(mesh->normals_[i+2]);
                }
                if (has_indices) {
                    combined_mesh.indices_.push_back(new_index);
                    combined_mesh.indices_.push_back(new_index+1);
                    combined_mesh.indices_.push_back(new_index+2);
                }
            }
        }
    }

    /**
     * Get a connected mesh by merging close vertices and removing triangles with
     * zero surface area. If you only would like to connect vertices, make sure
     * that the proximity threhsold <<< voxel size. If you would like to simplify
     * the mesh, chose a threshold greater or near the voxel size until you
     * reached the level of simpliciation desired.
     */
    void getConnectedMesh(
        Mesh& connected_mesh, 
        const Scalar approximate_vertex_proximity_threshold = 1e-10
    ) const
    {
        BlockIndexList mesh_indices;
        getAllocatedMeshes(&mesh_indices);

        AlignedVector<Mesh::ConstPtr> meshes;
        meshes.reserve(mesh_indices.size());
        for (const BlockIndex& block_index: mesh_indices)
        {
            meshes.push_back(getMeshConstPtr(block_index));
        }

        createConnectedMesh(meshes, connected_mesh, approximate_vertex_proximity_threshold);
    }

    size_t getNumberOfAllocatedMeshes() const 
    {
        return mesh_map_.size();
    }

    inline size_t getMemorySize() const 
    {
        size_t size_bytes = 0u;

        size_bytes += sizeof(block_size_);
        size_bytes += sizeof(block_size_inv_);

        for (const auto& index_mesh_pair: mesh_map_)
        {
            size_bytes += index_mesh_pair.second->getMemorySize();
        }

        return size_bytes;
    }

    void clear() {mesh_map_.clear();}

    Scalar blockSize() const {return block_size_;}
    Scalar blockSizeInv() const {return block_size_inv_;}

private:
    Scalar block_size_;
    Scalar block_size_inv_;
    MeshMap mesh_map_;
};
}