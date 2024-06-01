#pragma once

#include <sdf_generator/core/type.hpp>
#include <sdf_generator/core/color.hpp>

#include <opencv4/opencv2/core/mat.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace sdf_generator
{
using PointArray = AlignedVector<Point>;
using Vector3Array = AlignedVector<Vector3>;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGBL>;
} // namespace sdf_generator
