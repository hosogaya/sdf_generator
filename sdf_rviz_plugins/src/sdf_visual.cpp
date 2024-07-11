#include <sdf_rviz_plugins/sdf_visual.h>

namespace sdf_rviz_plugins
{

SdfVisual::SdfVisual(
    Ogre::SceneManager* scene_manager, 
    Ogre::SceneNode* parent_node, 
    std::string name_space
)
: scene_manager_(scene_manager), name_space_(name_space), 
is_enabled_(true), manual_object_(0), 
color_mode_(ColorMode::Rainbow), color_source_(ColorSource::Distance),
color_map_(new sdf_generator::RainbowColorMap)
{
    frame_node_ = parent_node->createChildSceneNode();
}

SdfVisual::~SdfVisual() {}

void SdfVisual::setMessage(
    const sdf_msgs::msg::EsdfLayer::ConstPtr& msg
)
{
    const sdf_generator::Layer<sdf_generator::EsdfVoxel>::Ptr layer = sdf_generator::msg2EsdfLayer(*msg);
    sdf_generator::BlockIndexList block_indices;
    layer->getAllAllocatedBlocks(block_indices);
    int height_index = std::floor(slice_height_*layer->blockSizeInv() + sdf_generator::kCoordinateEpsilon);
    int voxel_height_index = std::floor(
        (slice_height_ - height_index*layer->blockSize())
        *layer->voxelSizeInv() + sdf_generator::kCoordinateEpsilon);
    
    if (voxel_height_index < 0) 
    {
        std::cout << "[SdfVisual::setMessage] voxel height index takes minus value: " << voxel_height_index << std::endl;
        return;
    }

    size_t cols = getCols(block_indices, msg->voxels_per_side, height_index);
    size_t rows = getRows(block_indices, msg->voxels_per_side, height_index);

    if (!manual_object_)
    {
        static uint32_t count = 0;
        rviz_common::UniformStringStream ss;
        ss << "Mesh" << count++;
        manual_object_ = scene_manager_->createManualObject(ss.str());
        frame_node_->attachObject(manual_object_);

        ss << "Material";
        material_name_ = ss.str();
        material_ = Ogre::MaterialManager::getSingleton().create(material_name_, "rviz_rendering");
        material_->setReceiveShadows(false);
        material_->getTechnique(0)->setLightingEnabled(true);
        material_->setCullingMode(Ogre::CULL_NONE);
    }

    manual_object_->clear();
    if (cols == 0 || rows == 0) return;
    std::cout << "[SdfVisual::setMessage] cols: " << cols << ", rows: " << rows << std::endl;
    std::cout << "[SdfVisual::setMessage] setting color map status" << std::endl;
    // set color map status
    if (color_source_ == ColorSource::Distance)
    {
        float min_color_source = std::numeric_limits<float>::max();
        float max_color_source = std::numeric_limits<float>::min();
        int voxels_per_layer =layer->voxelsPerSide()*layer->voxelsPerSide();
        for (const auto& block_index: block_indices)
        {
            if (block_index.z() != height_index) continue;
            const auto block_ptr = layer->getBlockConstPtr(block_index);
            
            for (size_t i=voxels_per_layer*voxel_height_index; i<(voxels_per_layer)*(voxel_height_index+1);  ++i)
            {
                const auto& voxel = block_ptr->getConstVoxel(i);
                if (!voxel.observed_) continue;
                if (voxel.distance_ < min_color_source) min_color_source = voxel.distance_;
                if (voxel.distance_ > max_color_source) max_color_source = voxel.distance_; 
            }
        }
        color_map_->setMaxValue(max_color_source);
        color_map_->setMinValue(min_color_source);
    }
    else if (color_source_ == ColorSource::Gradient)
    {
        float min_color_source = std::numeric_limits<float>::max();
        float max_color_source = std::numeric_limits<float>::min();
        int voxels_per_layer =layer->voxelsPerSide()*layer->voxelsPerSide();
        for (const auto& block_index: block_indices)
        {
            if (block_index.z() != height_index) continue;
            const auto block_ptr = layer->getBlockConstPtr(block_index);
            
            for (size_t i=voxels_per_layer*voxel_height_index; i<(voxels_per_layer)*(voxel_height_index+1);  ++i)
            {
                const auto& voxel = block_ptr->getConstVoxel(i);
                if (!voxel.observed_) continue;
                float source_value = voxel.gradient_.dot(kGradientMultiplier);
                if (source_value < min_color_source) min_color_source = source_value;
                if (source_value > max_color_source) max_color_source = source_value;
            }
        }
        color_map_->setMaxValue(max_color_source);
        color_map_->setMinValue(min_color_source);
    }

    size_t vertices_num = 4 + 6*(cols*rows - cols - rows);
    manual_object_->estimateVertexCount(vertices_num);
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

    size_t object_counter = 0;
    size_t block_counter = 0;
    std::cout << "[SdfVisual::setMessage] creating objects" << std::endl;
    for (const auto& block_index: block_indices)
    {
        if (block_index.z() != height_index) continue;
        ++block_counter;
        auto block_ptr = layer->getBlockConstPtr(block_index);
        
        for (size_t i=0; i<block_ptr->voxelsPerSide(); ++i)
        {
            for (size_t j=0; j<block_ptr->voxelsPerSide(); ++j)
            {
                std::vector<Ogre::Vector3> vertices;
                std::vector<Ogre::ColourValue> colors;

                // square search and extracting the data
                for (size_t k=0; k<2; ++k)
                {
                    for (size_t l=0; l<2; ++l)
                    {
                        sdf_generator::VoxelIndex voxel_index(i+k, j+l, voxel_height_index);
                        if (voxel_index.x() < block_ptr->voxelsPerSide()
                        && voxel_index.y() < block_ptr->voxelsPerSide())
                        {
                            const sdf_generator::EsdfVoxel& voxel = block_ptr->getConstVoxel(voxel_index);
                            if (!voxel.observed_) continue;

                            sdf_generator::Point position = block_ptr->calCoordinate(voxel_index);
                            vertices.push_back(Ogre::Vector3(position.x(), position.y(), slice_height_));
                            colors.push_back(calColorValue(voxel));
                        }
                        else 
                        {
                            sdf_generator::BlockIndex new_block_index(block_index);
                            if (voxel_index.x() >= block_ptr->voxelsPerSide())
                            {
                                ++new_block_index.x();
                                voxel_index.x() = 0;
                            }
                            if (voxel_index.y() >= block_ptr->voxelsPerSide())
                            {
                                ++new_block_index.y();
                                voxel_index.y() = 0;
                            }
                            auto new_block_ptr = layer->getBlockConstPtr(new_block_index);
                            if (!new_block_ptr) continue;
                            const sdf_generator::EsdfVoxel& voxel = new_block_ptr->getConstVoxel(voxel_index);
                            if (!voxel.observed_) continue;

                            sdf_generator::Point position = new_block_ptr->calCoordinate(voxel_index);
                            vertices.push_back(Ogre::Vector3(position.x(), position.y(), slice_height_));
                            colors.push_back(calColorValue(voxel));
                        } // end of extract the data
                    }
                } // end of square search          
                
                // set objects
                if (vertices.size() > 2)
                {
                    Ogre::Vector3 normal = Ogre::Vector3::UNIT_Z;

                    for (size_t m=1; m<vertices.size() -1; ++m)
                    {
                        ++object_counter;
                        manual_object_->position(vertices[m-1]);
                        manual_object_->normal(normal);
                        manual_object_->colour(colors[m-1]);

                        manual_object_->position(vertices[m]);
                        manual_object_->normal(normal);
                        manual_object_->colour(colors[m]);

                        manual_object_->position(vertices[m+1]);
                        manual_object_->normal(normal);
                        manual_object_->colour(colors[m+1]);
                    }
                } // end of object settting
            }
        } // end of voxel indices loop 
    } // end of block indices loop

    manual_object_->end();
    material_->getTechnique(0)->setLightingEnabled(false);
    material_->getTechnique(0)->setColourWriteEnabled(true);

    std::cout << "[SdfVisual::setMessage] block num: " << block_counter << ", object num: " << object_counter << std::endl;
}

void SdfVisual::setEnabled(bool enabled)
{
    is_enabled_ = enabled;
}


size_t SdfVisual::getCols(
    const sdf_generator::BlockIndexList& blocks, 
    const size_t voxels_per_side, 
    const int height_index
) const
{
    int min_col = std::numeric_limits<int>::max();
    int max_col = std::numeric_limits<int>::min();
    bool exist = false;
    for (const auto& block: blocks)
    {
        if (block.z() != height_index) continue;
        exist = true;
        if (block.y() < min_col) min_col = block.y();
        if (block.y() > max_col) max_col = block.y();
    }

    if (!exist) return 0;
    return (max_col - min_col + 1)*voxels_per_side;
}

size_t SdfVisual::getRows(
    const sdf_generator::BlockIndexList& blocks, 
    const size_t voxels_per_side, 
    const int height_index
) const
{
    int min_row = std::numeric_limits<int>::max();
    int max_row = std::numeric_limits<int>::min();
    bool exist = false;
    for (const auto& block: blocks)
    {
        if (block.z() != height_index) continue;
        exist = true;
        if (block.x() < min_row) min_row = block.x();
        if (block.x() > max_row) max_row = block.x();
    }

    if (!exist) return 0;
    return (max_row - min_row + 1)*voxels_per_side;
}

Ogre::ColourValue SdfVisual::calColorValue(const sdf_generator::EsdfVoxel& voxel)
{
    float source_value;
    if (color_source_ == ColorSource::Distance)
    {
        source_value = voxel.distance_;
    }
    else if (color_source_ == ColorSource::Gradient)
    {
        source_value = voxel.gradient_.dot(kGradientMultiplier);
    }

    sdf_generator::Color color = color_map_->colorLookup(source_value);
    constexpr float color_conv_factor = 1.0/std::numeric_limits<uint8_t>::max();
    return Ogre::ColourValue(
        color_conv_factor*static_cast<float>(color.r_), 
        color_conv_factor*static_cast<float>(color.g_), 
        color_conv_factor*static_cast<float>(color.b_), 
        color_conv_factor*static_cast<float>(color.a_)
    );
}


}