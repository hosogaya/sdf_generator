#pragma once

#include <sdf_generator/mesh/type.hpp>
#include <sdf_generator/core/layer.hpp>
#include <sdf_generator/mesh/mesh_layer.hpp>
#include <sdf_generator/integrator/thread_safe_index.hpp>
#include <sdf_generator/mesh/marching_cubes.hpp>

#include <thread>

namespace sdf_generator
{
struct MeshIntegratorConfig
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool use_color_ = true;
    float min_weight_ =1e-4;

    size_t integrator_threads_ = std::thread::hardware_concurrency();
};

template <typename VoxelType>
class MeshIntegrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeshIntegrator(
        const MeshIntegratorConfig& config, 
        Layer<VoxelType>* sdf_layer,
        MeshLayer* mesh_layer
    )
    : config_(config), sdf_layer_mutable_(sdf_layer), 
    sdf_layer_const_(sdf_layer), mesh_layer_(mesh_layer)
    {
        initFromSdfLayer(*sdf_layer);

        cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0, 
                               0, 0, 1, 1, 0, 0, 1, 1, 
                               0, 0, 0, 0, 1, 1, 1, 1;

        if (config_.integrator_threads_ == 0) config_.integrator_threads_ = 1;
    }

    MeshIntegrator(
        const MeshIntegratorConfig& config, 
        Layer<VoxelType>& sdf_layer,
        MeshLayer* mesh_layer
    )
    : MeshIntegrator(config, &sdf_layer, mesh_layer)
    {}

    void initFromSdfLayer(const Layer<VoxelType>& sdf_layer)
    {
        voxel_size_ = sdf_layer.voxelSize();
        block_size_ = sdf_layer.blockSize();
        voxels_per_side_ = sdf_layer.voxelsPerSide();

        voxel_size_inv_ = sdf_layer.voxelSizeInv();
        block_size_inv_ = sdf_layer.blockSizeInv();
        voxels_per_side_inv_ = sdf_layer.voxelsPerSideInv();
    }

    // Generates mesh from the tsdf layer
    void generateMesh(bool only_mesh_updated_blocks, bool claer_updated_flag)
    {
        BlockIndexList all_tsdf_blocks;
        if (only_mesh_updated_blocks) sdf_layer_const_->getAllUpdatedBlocks(Update::kMesh, all_tsdf_blocks);
        else sdf_layer_const_->getAllAllocatedBlocks(all_tsdf_blocks);

        // Allocate all the mesh memory
        for (const BlockIndex& block_index: all_tsdf_blocks)
        {
            mesh_layer_->allocateMeshPtr(block_index);
        }

        std::unique_ptr<ThreadSafeIndex> index_getter(
            new MixedThreadSafeIndex(all_tsdf_blocks.size())
        );

        std::vector<std::thread> integration_threads;
        for (size_t i=0; i<config_.integrator_threads_; ++i)
        {
            integration_threads.emplace_back(
                &MeshIntegrator::generateMeshBlocksFunction, this, all_tsdf_blocks, 
                claer_updated_flag, index_getter.get()
            );
        }

        for (std::thread& thread: integration_threads) thread.join();
    }

    void generateMeshBlocksFunction(
        const BlockIndexList& all_tsdf_blocks, bool clear_updated_flag, 
        ThreadSafeIndex* index_getter
    )
    {
        size_t list_index;
        while (index_getter->getNextIndex(list_index))
        {
            const BlockIndex& block_index = all_tsdf_blocks[list_index];
            updateMeshForBlock(block_index);
            if (clear_updated_flag)
            {
                typename Block<VoxelType>::Ptr block = sdf_layer_mutable_->getBlockPtr(block_index);
                block->setUpdated(Update::kMesh, false);
            }
        }
    }

    virtual void updateMeshForBlock(const BlockIndex& block_index)
    {
        Mesh::Ptr mesh = mesh_layer_->getMeshPtr(block_index);

        // This block should already exist, otherwise it makes no sense to update
        // the mesh for it. ;)
        typename Block<VoxelType>::ConstPtr block = sdf_layer_const_->getBlockPtr(block_index);

        if (!block) return;

        extractBlockMesh(block, mesh);
        if (config_.use_color_)
        {
            updateMeshColor(*(block.get()), mesh.get());   
        }
        mesh->update_ = true;
    }

    void extractBlockMesh(
        typename Block<VoxelType>::ConstPtr block, Mesh::Ptr mesh
    )
    {
        IndexElement vps = block->voxelsPerSide();
        VertexIndex next_mesh_index = 0;

        VoxelIndex voxel_index;
        for (voxel_index.x()=0; voxel_index.x()<vps-1; ++voxel_index.x())
        {
            for (voxel_index.y()=0; voxel_index.y()<vps-1; ++voxel_index.y())
            {
                for (voxel_index.z()=0; voxel_index.z()<vps-1; ++voxel_index.z())
                {
                    Point coords = block->calCoordinate(voxel_index);
                    // extract Mesh Inside block
                    extractMeshInsideBlock(*block, voxel_index, coords, next_mesh_index, mesh.get());
                }
            }
        }

        // Max X plane
        // takes care of edge (x_max, y_max, z),
        // takes care of edge (x_max, y, z_max).
        voxel_index.x() = vps - 1;
        for (voxel_index.z()=0; voxel_index.z() < vps; ++voxel_index.z())
        {
            for (voxel_index.y()=0; voxel_index.y() < vps; ++voxel_index.y())
            {
                Point coords = block->calCoordinate(voxel_index);
                extractMeshOnBorder(
                    *block, voxel_index, coords, next_mesh_index, mesh.get()
                );
            }
        }

        // Max Y plane.
        // takes care of edge (x, y_max, z_max),
        // without corner (x_max, y_max, z_max).
        voxel_index.y() = vps - 1;
        for (voxel_index.z()=0; voxel_index.z() < vps; ++voxel_index.z())
        {
            for (voxel_index.x()=0; voxel_index.x() < vps - 1; ++voxel_index.x())
            {
                Point coords = block->calCoordinate(voxel_index);
                extractMeshOnBorder(
                    *block, voxel_index, coords, next_mesh_index, mesh.get()
                );
            }
        }

        // Max Z plane.
        voxel_index.z() = vps - 1;
        for (voxel_index.y()=0; voxel_index.y() < vps - 1; ++voxel_index.y())
        {
            for (voxel_index.x()=0; voxel_index.x() < vps - 1; ++voxel_index.x())
            {
                Point coords = block->calCoordinate(voxel_index);
                extractMeshOnBorder(
                    *block, voxel_index, coords, next_mesh_index, mesh.get()
                );
            }
        }
    }

    void extractMeshInsideBlock(
        const Block<VoxelType>& block, const VoxelIndex& index,
        const Point& coords, VertexIndex& next_mesh_index, Mesh* mesh
    )
    {
        Eigen::Matrix<Scalar, 3, 8> cube_coord_offset = cube_index_offsets_.cast<Scalar>()*voxel_size_;
        Eigen::Matrix<Scalar, 3, 8> corner_coords;
        Eigen::Matrix<Scalar, 8, 1> corner_sdf;
        bool all_neighbors_observed = true;

        for (unsigned int i=0; i<8; ++i)
        {
            VoxelIndex corner_index = index + cube_index_offsets_.col(i);
            const VoxelType& voxel = block.getConstVoxel(corner_index);

            if (!getSdfIfValid(voxel, config_.min_weight_, corner_sdf(i)))
            {
                all_neighbors_observed = false;
                break;
            }

            corner_coords.col(i) = coords + cube_coord_offset.col(i);
        }

        if (all_neighbors_observed)
            MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index, mesh);
    }

    void extractMeshOnBorder(
        const Block<VoxelType>& block, const VoxelIndex& index, 
        const Point& coords, VertexIndex& next_mesh_index, Mesh* mesh
    )
    {
        Eigen::Matrix<Scalar, 3, 8> cube_coord_offsets = cube_index_offsets_.cast<Scalar>() * voxel_size_;
        Eigen::Matrix<Scalar, 3, 8> corner_coords;
        Eigen::Matrix<Scalar, 8, 1> corner_sdf;
        bool all_neighbors_observed = true;
        corner_coords.setZero();
        corner_sdf.setZero();

        for (unsigned int i=0; i<8; ++i)
        {
            VoxelIndex corner_index = index + cube_index_offsets_.col(i);
            if (block.isValidVoxelIndex(corner_index))
            {
                if (!getSdfIfValid(voxel_size_, config_.min_weight_, (corner_sdf(i))))
                {
                    all_neighbors_observed = false;
                    break;
                }
                corner_coords.col(i) = coords + cube_coord_offsets.col(i);
            }
            else
            {
                // We have to access a different block.
                BlockIndex block_offset = BlockIndex::Zero();

                for (unsigned int j=0; j<3; ++j)
                {
                    if (corner_index(j) < 0) 
                    {
                        block_offset(j) = -1;
                        corner_index(j) = corner_index(j) + voxels_per_side_;
                    }
                    else if (corner_index(j) >= voxels_per_side_)
                    {
                        block_offset(j) = 1;
                        corner_index(j) = corner_index(j) - voxels_per_side_;
                    }
                }

                BlockIndex neighbor_index = block.blockIndex() + block_offset;
                if (sdf_layer_const_->hasBlock(neighbor_index))
                {
                    typename Block<VoxelType>::ConstPtr neighbor_block = sdf_layer_const_->getBlockPtr(neighbor_index);
                    const VoxelType& voxel = neighbor_block->getConstVoxel(corner_index);

                    if (!getSdfIfValid(voxel, config_.min_weight_, corner_sdf(i)))
                    {
                        all_neighbors_observed = false;
                        break;
                    }
                    corner_coords.col(i) = coords + cube_coord_offsets.col(i);
                }
                else 
                {
                    all_neighbors_observed = false;
                    break;
                }
            }
        }

        if (all_neighbors_observed)
            MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index, mesh);
    }

    void updateMeshColor(const Block<VoxelType>& block, Mesh* mesh)
    {
        mesh->colors_.clear();
        mesh->colors_.resize(mesh->indices_.size());

        // Use nearest neighbor search
        for (size_t i=0; i<mesh->vertices_.size(); ++i)
        {
            const Point& vertex = mesh->vertices_[i];
            VoxelIndex voxel_index = block.calVoxelIndex(vertex);

            if (block.isValidVoxelIndex(voxel_index))
            {
                const VoxelType& voxel = block.getConstVoxel(voxel_index);
                getColorIfValid(voxel, config_.min_weight_, mesh->colors_[i]);
            }
            else
            {
                const typename Block<VoxelType>::ConstPtr neighbor_block = sdf_layer_const_->getBlockPtr(vertex);
                const VoxelType& voxel = neighbor_block->getConstVoxel(vertex);
                getColorIfValid(voxel, config_.min_weight_, mesh->colors_[i]);
            }
        }
    }

protected:
    MeshIntegratorConfig config_;
    /**
     * Having both a const and a mutable pointer to the layer allows this
     * integrator to work both with a const layer (in case you don't want to clear
     * the updated flag) and mutable layer (in case you do want to clear the
     * updated flag).
     */
    Layer<VoxelType>* sdf_layer_mutable_;
    const Layer<VoxelType>* sdf_layer_const_;

    MeshLayer* mesh_layer_;

    Scalar voxel_size_;
    size_t voxels_per_side_;
    Scalar block_size_;

    Scalar voxel_size_inv_;
    Scalar voxels_per_side_inv_;
    Scalar block_size_inv_;

    Eigen::Matrix<int, 3, 8> cube_index_offsets_;
};
}