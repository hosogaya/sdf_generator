#pragma once

#include <sdf_generator/core/layer.hpp>
#include <sdf_msgs/msg/layer.hpp>

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

        for (int i=0; i<block_ptr->numVoxels(); ++i)
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
}