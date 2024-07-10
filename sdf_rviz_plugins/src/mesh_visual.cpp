#include <sdf_rviz_plugins/mesh_visual.h>

namespace sdf_rviz_plugins
{
unsigned int MeshVisual::instance_counter_ = 0;

MeshVisual::MeshVisual(
    Ogre::SceneManager* scene_manager, 
    Ogre::SceneNode* parent_node,
    std::string name_space
)
: scene_manager_(scene_manager), name_space_(name_space), is_enabled_(true)
{
    frame_node_ = parent_node->createChildSceneNode();
    instance_number_ = instance_number_++; // 0, 1, 2,...
}

MeshVisual::~MeshVisual()
{
    for (auto& ogre_object_pair: object_map_)
    {
        scene_manager_->destroyManualObject(ogre_object_pair.second);
    }
}

void MeshVisual::setPose(
    const Ogre::Vector3& position, const Ogre::Quaternion& orientation
)
{
    frame_node_->setPosition(position);
    frame_node_->setOrientation(orientation);
}

void MeshVisual::setMessage(
    const sdf_msgs::msg::Mesh::ConstPtr& msg
)
{
    for (const sdf_msgs::msg::MeshBlock& mesh_block: msg->mesh_blocks)
    {
        const sdf_generator::BlockIndex index
        (
            mesh_block.index[0], 
            mesh_block.index[1],
            mesh_block.index[2]
        );

        size_t vertex_index = 0;
        sdf_generator::Mesh mesh;
        mesh.indices_.reserve(mesh_block.x.size());
        mesh.vertices_.reserve(mesh_block.x.size());

        // translate vertex data from message to voxblox mesh
        for (size_t i=0; i<mesh_block.x.size(); ++i)
        {
            // Each vertex is given as its distance from the blocks origin in units of
            // (2*block_size), see mesh_vis.h for the slightly convoluted
            // justification of the 2. 
            constexpr float point_conv_factor = 2.0/std::numeric_limits<uint16_t>::max();
            const float mesh_x = (static_cast<float>(mesh_block.x[i])*point_conv_factor
                                + static_cast<float>(index[0]))*msg->block_edge_length;
            const float mesh_y = (static_cast<float>(mesh_block.y[i])*point_conv_factor
                                + static_cast<float>(index[1]))*msg->block_edge_length;
            const float mesh_z = (static_cast<float>(mesh_block.z[i])*point_conv_factor
                                + static_cast<float>(index[2]))*msg->block_edge_length;
        
            mesh.indices_.push_back(vertex_index++);
            mesh.vertices_.emplace_back(mesh_x, mesh_y, mesh_z);
        }

        // calculate normals
        mesh.normals_.reserve(mesh.vertices_.size());
        for (size_t i=0; i<mesh.vertices_.size(); i+=3)
        {
            const sdf_generator::Vector3 dir0 = mesh.vertices_[i] - mesh.vertices_[i+1];
            const sdf_generator::Vector3 dir1 = mesh.vertices_[i] - mesh.vertices_[i+2];
            const sdf_generator::Vector3 normal = dir0.cross(dir1).normalized();

            mesh.normals_.push_back(normal);
            mesh.normals_.push_back(normal);
            mesh.normals_.push_back(normal);
        }

        // add color information
        mesh.colors_.reserve(mesh.vertices_.size());
        const bool has_color = mesh_block.x.size() == mesh_block.r.size();
        if (has_color)
        {
            for (size_t i=0; i<mesh_block.x.size(); ++i)
            {
                sdf_generator::Color color(
                    mesh_block.r[i], 
                    mesh_block.g[i], 
                    mesh_block.b[i], 
                    mesh_block.a[i]
                );
                mesh.colors_.push_back(color);
            }
        }
        else
        {
            for (size_t i=0; i<mesh_block.x.size(); ++i)
            {
                sdf_generator::Color color(
                    std::numeric_limits<uint8_t>::max()*(mesh.normals_[i].x()*0.5f + 0.5f),
                    std::numeric_limits<uint8_t>::max()*(mesh.normals_[i].y()*0.5f + 0.5f),
                    std::numeric_limits<uint8_t>::max()*(mesh.normals_[i].z()*0.5f + 0.5f),
                    std::numeric_limits<uint8_t>::max()
                );
                mesh.colors_.push_back(color);
            }
        }

        // connect mesh
        sdf_generator::Mesh connected_mesh;
        sdf_generator::createConnectedMesh(mesh, connected_mesh);

        // create ogre object
        Ogre::ManualObject* ogre_object;
        const auto it = object_map_.find(index);
        if (it != object_map_.end())
        {
            // delete empty mesh blocks
            if (mesh_block.x.size() == 0)
            {
                // std::cout << "[MeshVisual] Mesh block size is zero" << std::endl;
                scene_manager_->destroyManualObject(it->second);
                object_map_.erase(it);
                continue;
            }
            // else
            ogre_object = it->second;
            ogre_object->clear();
        }
        else 
        {
            std::string object_name = std::to_string(index.x()) + std::string(" ") + 
                                      std::to_string(index.y()) + std::string(" ") +
                                      std::to_string(index.z()) + std::string(" ") + 
                                      std::to_string(instance_number_) + 
                                      std::string(" ") + name_space_;

            ogre_object = scene_manager_->createManualObject(object_name);
            object_map_.insert(std::make_pair(index, ogre_object));
            if (!is_enabled_) 
            {
                std::cout << "[MeshVisual] not enabled now" << std::endl;
                ogre_object->setVisible(false);
            }
            frame_node_->attachObject(ogre_object);
        }

        ogre_object->estimateVertexCount(connected_mesh.vertices_.size());
        ogre_object->estimateIndexCount(connected_mesh.indices_.size());

        
        std::string material_name("SdfMaterial");
        ogre_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST, "SdfMaterials");

        for (size_t i=0; i<connected_mesh.vertices_.size(); ++i)
        {
            // note calling position changes what vertex the color and normal calls point to
            ogre_object->position(
                connected_mesh.vertices_[i].x(),
                connected_mesh.vertices_[i].y(),
                connected_mesh.vertices_[i].z()
            );

            ogre_object->normal(
                connected_mesh.normals_[i].x(),
                connected_mesh.normals_[i].y(),
                connected_mesh.normals_[i].z()
            );

            constexpr float color_conv_factor = 1.0/std::numeric_limits<uint8_t>::max();
            ogre_object->colour(
                color_conv_factor*static_cast<float>(connected_mesh.colors_[i].r_),
                color_conv_factor*static_cast<float>(connected_mesh.colors_[i].g_),
                color_conv_factor*static_cast<float>(connected_mesh.colors_[i].b_),
                // color_conv_factor*static_cast<float>(connected_mesh.colors_[i].a_)
                color_conv_factor*255.0
            );
        }
        // needed for anything other than flat rendering
        for (const auto& index: connected_mesh.indices_) ogre_object->index(index);

        ogre_object->end();
    }
    // std::cout << "[MeshVisual] object map size: " << object_map_.size() << std::endl;
}

void MeshVisual::setEnabled(bool enabled)
{
    if (enabled && !is_enabled_)
    {
        // new enable
        for (auto& manual_object : object_map_) manual_object.second->setVisible(true);
    }
    else if (!enabled && is_enabled_)
    {
        for (auto& manual_object: object_map_) manual_object.second->setVisible(false);
    }
    is_enabled_ = enabled;
}

}