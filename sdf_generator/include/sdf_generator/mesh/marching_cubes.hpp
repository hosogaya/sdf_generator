#pragma once

#include <sdf_generator/mesh/mesh.hpp>

namespace sdf_generator
{
/**
 * Performs the marching cubes algorithm to generate a mesh layer from a TSDF.
 * Implementation taken from Open Chisel
 * https://github.com/personalrobotics/OpenChisel
 */
class MarchingCubes
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int kTriangleTable[256][16];
    static const int kEdgeIndexPairs[12][2];

    MarchingCubes() {}
    virtual ~MarchingCubes() {}

    static void meshCube(
        const Eigen::Matrix<Scalar, 3, 8>& vertex_coords,
        const Eigen::Matrix<Scalar, 8, 1>& vertex_sdf,
        TriangleList& triangles
    )
    {
        const int index = calVertexConfiguration(vertex_sdf);

        Eigen::Matrix<Scalar, 3, 12> edge_coords;
        interpolateEdgeVertices(vertex_coords, vertex_sdf, edge_coords);

        const int* table_row = kTriangleTable[index];

        int edge_index = 0;
        int table_col = 0;
        // while table_row[table_col] != -1
        while ((edge_index = table_row[table_col]) != -1)
        {
            Triangle triangle;
            triangle.col(0) = edge_coords.col(edge_index);
            edge_index = table_row[table_col + 1];
            triangle.col(1) = edge_coords.col(edge_index);
            edge_index = table_row[table_col + 2];
            triangle.col(2) = edge_coords.col(edge_index);

            triangles.push_back(triangle);
            table_col += 3;
        }
    }

    static void meshCube(
        const Eigen::Matrix<Scalar, 3, 8>& vertex_coords,
        const Eigen::Matrix<Scalar, 8, 1>& vertex_sdf, 
        VertexIndex& next_index, Mesh* mesh
    )
    {
        const int index = calVertexConfiguration(vertex_sdf);

        // No edges in this cube.
        if (index == 0) return;
        // for each cube, 12 edges, 12 vertex
        Eigen::Matrix<Scalar, 3, 12> edge_vertex_coords;
        interpolateEdgeVertices(vertex_coords, vertex_sdf, edge_vertex_coords);

        const int* table_row = kTriangleTable[index];

        int table_col = 0;
        while (table_row[table_col] != -1) {
            mesh->vertices_.emplace_back(
                edge_vertex_coords.col(table_row[table_col + 2]));
            mesh->vertices_.emplace_back(
                edge_vertex_coords.col(table_row[table_col + 1]));
            mesh->vertices_.emplace_back(
                edge_vertex_coords.col(table_row[table_col]));
            mesh->indices_.push_back(next_index);
            mesh->indices_.push_back(next_index+1);
            mesh->indices_.push_back(next_index+2);
            const Point& p0 = mesh->vertices_[next_index];
            const Point& p1 = mesh->vertices_[next_index+1];
            const Point& p2 = mesh->vertices_[next_index+2];
            Point px = (p1 - p0);
            Point py = (p2 - p0);
            Point n = px.cross(py).normalized();
            mesh->normals_.push_back(n);
            mesh->normals_.push_back(n);
            mesh->normals_.push_back(n);
            next_index += 3;
            table_col += 3;
        }
        std::cout << "[meshCube] teh mesh vertices size: " << mesh->vertices_.size() << std::endl;
    }

    // if a bit takes 1, it takes minus sdf value
    static int calVertexConfiguration(
        const Eigen::Matrix<Scalar, 8, 1>& vertex_sdf
    )
    {
        return (vertex_sdf(0) < 0 ? (1 << 0) : 0) |
               (vertex_sdf(1) < 0 ? (1 << 1) : 0) |
               (vertex_sdf(2) < 0 ? (1 << 2) : 0) |
               (vertex_sdf(3) < 0 ? (1 << 3) : 0) |
               (vertex_sdf(4) < 0 ? (1 << 4) : 0) |
               (vertex_sdf(5) < 0 ? (1 << 5) : 0) |
               (vertex_sdf(6) < 0 ? (1 << 6) : 0) |
               (vertex_sdf(7) < 0 ? (1 << 7) : 0);
    }

    static void interpolateEdgeVertices(
        const Eigen::Matrix<Scalar, 3, 8>& vertex_coords,
        const Eigen::Matrix<Scalar, 8, 1>& vertex_sdf, 
        Eigen::Matrix<Scalar, 3, 12>& edge_coords 
    )
    {
        for (size_t i=0; i<12; ++i)
        {
            const int* pairs = kEdgeIndexPairs[i];
            const int edge0 = pairs[0];
            const int edge1 = pairs[1];

            // Only interpolate along edges where there is a zero crossing.
            if ((vertex_sdf(edge0) < 0 && vertex_sdf(edge1) >= 0) ||
                (vertex_sdf(edge0) >= 0 && vertex_sdf(edge1) < 0))
            {
                edge_coords.col(i) = interpolateVertex(
                    vertex_coords.col(edge0), vertex_coords.col(edge1),
                    vertex_sdf(edge0), vertex_sdf(edge1)
                );

            }
        }
    }

    /**
     * Performs linear interpolation on two cube corners to find the approximate
     * zero crossing (surface) value.
     */
    static inline Point interpolateVertex(
        const Point& vertex1, const Point& vertex2, float sdf1, float sdf2
    ) 
    {
        static constexpr Scalar kMinSdfDifference = 1e-6;
        const Scalar sdf_diff = sdf1 - sdf2;
        // Only compute the actual interpolation value if the sdf_difference is not
        // too small, this is to counteract issues with floating point precision.
        if (std::abs(sdf_diff) >= kMinSdfDifference) 
        {
            const Scalar t = sdf1 / sdf_diff;
            return Point(vertex1 + t * (vertex2 - vertex1));
        } 
        else 
        {
            return Point(0.5 * (vertex1 + vertex2));
        }
    }
};
}