#pragma once

#include <cstdint>
#include <memory>

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/point_cloud/type.hpp>
#include <sdf_generator/mesh/type.hpp>

namespace sdf_generator
{
struct Mesh
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Mesh>;
    using ConstPtr = std::shared_ptr<const Mesh>;

    static constexpr Scalar kInvalidBlockSize = -1.0;

    Mesh() 
    : block_size_(kInvalidBlockSize), origin_(Point::Zero())
    {}
    Mesh(Scalar block_size, const Point& origin) 
    : block_size_(block_size), origin_(origin)
    {}

    virtual ~Mesh() {}

    inline bool hasVertices() const {return !vertices_.empty();}
    inline bool hasNormals() const {return !normals_.empty();}
    inline bool hasColors() const {return !colors_.empty();}
    inline bool hasTriangle() const {return indices_.empty();}

    inline size_t size() const {return vertices_.size();}

    inline size_t getMemorySize() const 
    {
        size_t size_bytes = 0u;
        size_bytes += sizeof(PointArray) + vertices_.size()*sizeof(Point);
        size_bytes += sizeof(Vector3Array) + normals_.size()*sizeof(Vector3);
        size_bytes += sizeof(ColorArray) + colors_.size()*sizeof(Color);
        size_bytes += sizeof(VertexIndexList) + indices_.size()*sizeof(VertexIndex);

        size_bytes += sizeof(block_size_);
        size_bytes += sizeof(origin_);
        size_bytes += sizeof(update_);

        return size_bytes;
    }

    inline void clear() 
    {
        vertices_.clear();
        normals_.clear();
        colors_.clear();
        indices_.clear();
    }

    inline void clearTriangles() {indices_.clear();}
    inline void clearNormals() {normals_.clear();}
    inline void clearColors() {colors_.clear();}

    inline void resize(
        const size_t size, const bool has_normals = true, 
        const bool has_colors = true, const bool has_indices = true
    )
    {
        vertices_.resize(size);
        if (has_normals) normals_.resize(size);
        if (has_colors) colors_.resize(size);
        if (has_indices) indices_.resize(size);
    }

    inline void reserve(
        const size_t size, const bool has_normals = true, 
        const bool has_colors = true, const bool has_indices = true
    )
    {
        vertices_.reserve(size);
        if (has_normals) normals_.reserve(size);
        if (has_colors) colors_.reserve(size);
        if (has_indices) indices_.reserve(size);
    }

    void colorizeMesh(const Color& new_color)
    {
        colors_.clear();
        colors_.resize(vertices_.size(), new_color);
    }

    bool concatenateMesh(const Mesh& other_mesh)
    {
        if (other_mesh.hasColors() != hasColors()) return false;
        if (other_mesh.hasNormals() != hasNormals()) return false;
        if (other_mesh.hasTriangle() != hasTriangle()) return false;

        reserve(
            size() + other_mesh.size(), hasNormals(), hasColors(), hasTriangle()
        );

        const size_t num_vertices = vertices_.size();

        for (const Point& vertex: other_mesh.vertices_)
        {
            vertices_.push_back(vertex);
        }
        for (const Vector3& normal: other_mesh.normals_)
        {
            normals_.push_back(normal);
        }
        for (const Color& color: other_mesh.colors_)
        {
            colors_.push_back(color);
        }
        for (const size_t index: other_mesh.indices_)
        {
            indices_.push_back(index+ num_vertices);
        }

        return true;
    }

    PointArray vertices_;
    VertexIndexList indices_;
    Vector3Array normals_;
    ColorArray colors_;
    Scalar block_size_;
    Point origin_;

    bool update_;
};
}