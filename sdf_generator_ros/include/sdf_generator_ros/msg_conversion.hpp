#pragma once

#include <sdf_generator/core/layer.hpp>
#include <sdf_msgs/msg/layer.hpp>
#include <sdf_generator/mesh/mesh_layer.hpp>
#include <sdf_msgs/msg/mesh.hpp>

namespace sdf_generator
{
inline sdf_msgs::msg::Layer::UniquePtr toMsg(const Layer<TsdfVoxel>::Ptr layer)
{
    sdf_msgs::msg::Layer::UniquePtr msg(new sdf_msgs::msg::Layer);

    BlockIndexList indexes;
    layer->getAllAllocatedBlocks(indexes);
    int voxels_num = layer->voxelsPerSide()*layer->voxelsPerSide()*layer->voxelsPerSide();
    msg->voxel_size = layer->voxelSize();
    msg->voxels_per_side = layer->voxelsPerSide();
    msg->blocks.reserve(indexes.size());

    for (const auto& index: indexes)
    {
        sdf_msgs::msg::Block block_msg;
        auto block_ptr = layer->getBlockPtr(index);
        
        block_msg.x_index = index.x();
        block_msg.y_index = index.y();
        block_msg.z_index = index.z();

        block_msg.voxels.reserve(voxels_num);
        for (int i=0; i<voxels_num; ++i)
        {
            const auto& voxel = block_ptr->getConstVoxel(i);
            sdf_msgs::msg::Voxel voxel_msg;
            if (voxel.occupied_)
            {
                voxel_msg.has_data = true;
                voxel_msg.distance = voxel.distance_;
                voxel_msg.weight = voxel.weight_;
                voxel_msg.gradient.x = voxel.gradient_.x();
                voxel_msg.gradient.y = voxel.gradient_.y();
                voxel_msg.gradient.z = voxel.gradient_.z();
                voxel_msg.color.a = voxel.color_.a_;
                voxel_msg.color.r = voxel.color_.r_;
                voxel_msg.color.g = voxel.color_.g_;
                voxel_msg.color.b = voxel.color_.b_;
            }
            block_msg.voxels.push_back(voxel_msg);
        }
        msg->blocks.push_back(block_msg);
    }
    return msg;
}

inline Layer<TsdfVoxel>::Ptr fromMsg(sdf_msgs::msg::Layer& msg)
{
    Layer<TsdfVoxel>::Ptr layer = std::make_shared<Layer<TsdfVoxel>>(msg.voxel_size, msg.voxels_per_side);

    for (const auto& block: msg.blocks)
    {
        BlockIndex index(block.x_index, block.y_index, block.z_index);
        Point origin = calOrigin(index, layer->blockSize());
        Block<TsdfVoxel>::Ptr block_ptr = std::make_shared<Block<TsdfVoxel>>(layer->voxelSize(), layer->voxelsPerSide(), origin);

        for (size_t i=0; i<block_ptr->numVoxels(); ++i)
        {
            auto voxel = block_ptr->getVoxel(i);
            voxel.occupied_ = block.voxels[i].has_data;
            if (voxel.occupied_)
            {
                voxel.distance_ = block.voxels[i].distance;
                voxel.weight_ = block.voxels[i].weight;
                voxel.gradient_.x() = block.voxels[i].gradient.x;
                voxel.gradient_.y() = block.voxels[i].gradient.y;
                voxel.gradient_.z() = block.voxels[i].gradient.z;

                voxel.color_.a_ = block.voxels[i].color.a;
                voxel.color_.r_ = block.voxels[i].color.r;
                voxel.color_.g_ = block.voxels[i].color.g;
                voxel.color_.b_ = block.voxels[i].color.b;
            }
        }
        layer->insertBlock(std::make_pair(index, block_ptr));
    }

    return layer;
}


/**************************
 * Mesh conversion
 ***************************/

enum ColorMode {
  kColor = 0,
  kHeight,
  kNormals,
  kGray,
};


inline ColorMode getColorMode(const std::string& color_mode_string)
{
    if (color_mode_string.empty()) return ColorMode::kColor;
    else
    {
        if (color_mode_string == "color" || color_mode_string == "colors")
            return ColorMode::kColor;
        else if (color_mode_string == "height")
            return ColorMode::kHeight;
        else if (color_mode_string == "normals")
            return ColorMode::kNormals;
        else 
            return ColorMode::kGray;
    }
}

inline void colorToMsg(
    const Color& color, std_msgs::msg::ColorRGBA& color_msg
)
{
    color_msg.r = color.r_ / 255.0;
    color_msg.g = color.g_ / 255.0;
    color_msg.b = color.b_ / 255.0;
    color_msg.a = color.a_ / 255.0;
}

inline void heightColor(
    const Point& vertex, std_msgs::msg::ColorRGBA& color_msg
)
{
    const double min_z = -1;
    const double max_z = 10;
    double mapped_height = std::min<Scalar>(
        std::max<Scalar>(
            (vertex.z() - min_z)/(max_z - min_z), 0.0
        ), 1.0
    );
    colorToMsg(Color::convert2Rainbow(mapped_height), color_msg);
}

inline void normalColor(
    const Vector3& normal, std_msgs::msg::ColorRGBA& color_msg
)
{
    color_msg.r = normal.x()*0.5 + 0.5;
    color_msg.g = normal.y()*0.5 + 0.5;
    color_msg.b = normal.z()*0.5 + 0.5;
    color_msg.a = 1.0;
}

inline std_msgs::msg::ColorRGBA getVertexColor(
    const Mesh::ConstPtr& mesh, const ColorMode& color_mode, 
    const size_t index
)
{
    std_msgs::msg::ColorRGBA color_msg;
    switch (color_mode)
    {
        case kColor:
            colorToMsg(mesh->colors_[index], color_msg);
            break;
        case kHeight:
            heightColor(mesh->vertices_[index], color_msg);
            break;
        case kNormals:
            normalColor(mesh->normals_[index], color_msg);
            break;
        case kGray:
            color_msg.r = 0.5f;
            color_msg.g = 0.5f;
            color_msg.b = 0.5f;
            color_msg.a = 1.0f;
    }

    return color_msg;
}


inline void generateMeshMsg(
    MeshLayer* mesh_layer, ColorMode color_mode,
    sdf_msgs::msg::Mesh* mesh_msg
)
{
    // time stamp

    BlockIndexList mesh_indices;
    mesh_layer->getAllUpdatedMeshes(&mesh_indices);

    std::cout << "[generateMeshMsg] Mesh index size: " << mesh_indices.size() << std::endl;

    mesh_msg->block_edge_length = mesh_layer->blockSize();
    mesh_msg->mesh_blocks.reserve(mesh_indices.size());

    for (const BlockIndex& block_index: mesh_indices)
    {
        Mesh::Ptr mesh = mesh_layer->getMeshPtr(block_index);

        sdf_msgs::msg::MeshBlock mesh_block;
        mesh_block.index[0] = block_index.x();
        mesh_block.index[1] = block_index.y();
        mesh_block.index[2] = block_index.z();
    
        mesh_block.x.reserve(mesh->vertices_.size());
        mesh_block.y.reserve(mesh->vertices_.size());
        mesh_block.z.reserve(mesh->vertices_.size());

        // normal coloring is used by Rviz plugin by default, so no need to send it
        if (color_mode != kNormals)
        {
            mesh_block.r.reserve(mesh->vertices_.size());
            mesh_block.g.reserve(mesh->vertices_.size());
            mesh_block.b.reserve(mesh->vertices_.size());
        }
        for (size_t i=0; i<mesh->vertices_.size(); ++i)
        {
            // We convert from an absolute global frame to a normalized local frame.
            // Each vertex is given as its distance from the blocks origin in units of
            // (2*block_size). This results in all points obtaining a value in the
            // range 0 to 1. To enforce this 0 to 1 range we technically only need to
            // divide by (block_size + voxel_size). The + voxel_size comes from the
            // way marching cubes allows the mesh to interpolate between this and a
            // neighboring block. We instead divide by (block_size + block_size) as
            // the mesh layer has no knowledge of how many voxels are inside a block.
            
            const Point normalized_vertex = 0.5f*(mesh_layer->blockSizeInv()*mesh->vertices_[i] - block_index.cast<Scalar>());

            if (normalized_vertex.squaredNorm() > 1.0f) 
            {
                std::cout << "[generateMeshMsg] squred norm of normalized vertex is too large: " << normalized_vertex.squaredNorm() << std::endl;
                continue;
            }
            // convert to unit16_t fixed point representation
            mesh_block.x.push_back(std::numeric_limits<uint16_t>::max()*normalized_vertex.x());
            mesh_block.y.push_back(std::numeric_limits<uint16_t>::max()*normalized_vertex.y());
            mesh_block.z.push_back(std::numeric_limits<uint16_t>::max()*normalized_vertex.z());

            if (color_mode != kNormals)
            {
                const std_msgs::msg::ColorRGBA color_msg = getVertexColor(mesh, color_mode, i);
                mesh_block.r.push_back(std::numeric_limits<uint8_t>::max()*color_msg.r);
                mesh_block.g.push_back(std::numeric_limits<uint8_t>::max()*color_msg.g);
                mesh_block.b.push_back(std::numeric_limits<uint8_t>::max()*color_msg.b);
            }
        }

        mesh_msg->mesh_blocks.push_back(mesh_block);

        // delete empty mesh blocks after sending them
        if (!mesh->hasVertices())
        {
            std::cout << "[generateMeshMsg] the mesh does not have vertices" << std::endl;
            mesh_layer->removeMesh(block_index);
        }

        mesh->update_ = false;
    }
}

}