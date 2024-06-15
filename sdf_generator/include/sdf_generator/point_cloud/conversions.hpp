#pragma once

#include <sdf_generator/point_cloud/type.hpp>
#include <sdf_generator/point_cloud/color_maps/color_map.hpp>

namespace sdf_generator
{

template <typename PclPointType>
inline bool isValidPoint(const PclPointType& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

template <typename PclPointType>
Color convertColor(const PclPointType& point, const ColorMap::Ptr& color_map);

template <>
inline Color convertColor(const pcl::PointXYZRGBL& point, const ColorMap::Ptr& color_map)
{
    return Color(point.r, point.g, point.b);
}

template <>
inline Color convertColor(const pcl::PointXYZRGB& point, const ColorMap::Ptr& color_map)
{
    return Color(point.r, point.g, point.b);
}

template <>
inline Color convertColor(const pcl::PointXYZI& point, const ColorMap::Ptr& color_map)
{
    return color_map->colorLookup(point.intensity);
}

template <>
inline Color convertColor(const pcl::PointXYZ& point, const ColorMap::Ptr& color_map)
{
    return color_map->colorLookup(0);
}

template <typename PclPointType>
inline void convertPointCloud(
    const pcl::PointCloud<PclPointType>& point_cloud, 
    const ColorMap::Ptr color_map,
    PointArray& points, ColorArray& colors
)
{
    points.reserve(point_cloud.size());
    colors.reserve(point_cloud.size());

    for (size_t i=0; i<point_cloud.size(); ++i)
    {
        if (!isValidPoint(point_cloud[i])) continue;
        
        points.emplace_back(point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);
        colors.emplace_back(convertColor<PclPointType>(point_cloud[i], color_map));
    }
}

}