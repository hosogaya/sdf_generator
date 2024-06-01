#include <sdf_generator/point_cloud/point_cloud_processor.hpp>

namespace sdf_generator
{
PointCloudProcessor::PointCloudProcessor(
    const int width, const int height, 
    const Scalar min_d, const Scalar min_z, 
    const Scalar depth_smooth_thres_ratio, 
    const bool is_loop)
: width_(width), height_(height), 
min_d_(min_d), min_z_(min_z),
depth_smooth_thres_ratio_(depth_smooth_thres_ratio), 
is_loop_(is_loop)
{}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::process(PointArray& point, ColorArray& color, Vector3Array& normal)
{
    cv::Mat vertex_image = cv::Mat::zeros(height_, width_, CV_32FC3);
    cv::Mat depth_image(vertex_image.size(), CV_32FC1, -1.0);
    cv::Mat color_image = cv::Mat::zeros(vertex_image.size(), CV_8UC3);

    projectPointCloudToImage(point, color, vertex_image, depth_image, color_image, min_d_, min_z_);
    cv::Mat normal_image = calNormalImage(vertex_image, depth_image);

    point = extractPointArray(vertex_image, depth_image);
    color = extractColorArray(color_image, depth_image);
    normal = extractNormalArray(normal_image, depth_image);

    return;
}


void PointCloudProcessor::projectPointCloudToImage
(
    const PointArray& point, const ColorArray& color, 
    cv::Mat& vertex_map, cv::Mat& depth_image, 
    cv::Mat& color_image, Scalar min_d, Scalar min_z
)
{
    for (size_t i=0; i<point.size(); ++i)
    {
        int u, v;
        Scalar depth;
        depth = projectPointToImage(point[i], u, v);
        
        // valid 
        if (depth > min_d && point[i].z() > min_z)
        {
            Scalar old_depth = depth_image.at<Scalar>(v, u);

            if (old_depth <= 0.0 || old_depth > depth)
            {
                for (int k=0; k<3; ++k)
                {
                    vertex_map.at<VertexImageType>(v, u)[k] = point[i](k);
                }
                depth_image.at<float>(v, u) = depth;
                color_image.at<ColorImageType>(v, u)[0] = color[i].b_;
                color_image.at<ColorImageType>(v, u)[1] = color[i].g_;
                color_image.at<ColorImageType>(v, u)[2] = color[i].r_;
            }
        }
    }
    return ;
}

cv::Mat PointCloudProcessor::calNormalImage(const cv::Mat& vertex_image, const cv::Mat& depth_image)
{
    cv::Mat normal_image(depth_image.size(), CV_32FC3, 0.0);
    for (int u=0; u < width_; ++u)
    {
        for (int v=0; v < height_; ++v)
        {
            Point p(
                vertex_image.at<VertexImageType>(v, u)[0],
                vertex_image.at<VertexImageType>(v, u)[1],
                vertex_image.at<VertexImageType>(v, u)[2]
            );

            float depth = depth_image.at<Scalar>(v, u);
            float sign = 1.0;

            if (depth <= 0.0) continue;
            
            // get neighbor
            int nx_u;
            if (u == width_ - 1)
            {
                if (is_loop_) nx_u = 0;
                else nx_u = u - 1;
            }
            else nx_u = u + 1;

            Scalar depth_nx = depth_image.at<DepthImageType>(v, nx_u);
            if (depth_nx < 0.0) continue;
            if (std::abs(depth_nx - depth) > depth_smooth_thres_ratio_*depth) continue;

            int ny_v;
            if (v == width_ - 1)
            {
                if (is_loop_) ny_v = 0;
                else ny_v = v - 1;
            }
            else ny_v = v + 1;

            Scalar depth_ny = depth_image.at<DepthImageType>(ny_v, u);
            if (depth_ny < 0.0) continue;
            if (std::abs(depth_ny - depth) > depth_smooth_thres_ratio_*depth) continue;

            Point nx(
                vertex_image.at<VertexImageType>(v, nx_u)[0],
                vertex_image.at<VertexImageType>(v, nx_u)[1],
                vertex_image.at<VertexImageType>(v, nx_u)[2]
            );

            Point ny(
                vertex_image.at<VertexImageType>(ny_v, u)[0],
                vertex_image.at<VertexImageType>(ny_v, u)[1],
                vertex_image.at<VertexImageType>(ny_v, u)[2]
            );

            Vector3 dx = nx - p;
            Vector3 dy = ny - p;

            Vector3 normal = (dx.cross(dy).normalized()*sign);
            NormalImageType& n = normal_image.at<NormalImageType>(v, u);
            for (int k=0; k<3; ++k)
            {
                n[k] = normal(k);
            }
        }
    }

    return normal_image;
}


PointArray PointCloudProcessor::extractPointArray(const cv::Mat& vertex_image, const cv::Mat& depth_image) const
{
    PointArray points;
    for (int v=0; v<vertex_image.rows; ++v)
    {
        for (int u=0; u<vertex_image.cols; ++u)
        {
            if (depth_image.at<DepthImageType>(v, u) <=0.0) continue;
            const VertexImageType& vertex = vertex_image.at<VertexImageType>(v, u);
            points.emplace_back(vertex[0], vertex[1], vertex[2]);
        }
    }
    return points;
}

ColorArray PointCloudProcessor::extractColorArray(const cv::Mat& color_image,  const cv::Mat& depth_image) const
{
    ColorArray colors;
    for (int v=0; v<color_image.rows; ++v)
    {
        for (int u=0; u<color_image.cols; ++u)
        {
            if (depth_image.at<DepthImageType>(v, u) <=0.0) continue;
            const ColorImageType& color = color_image.at<ColorImageType>(v, u);
            colors.emplace_back(color[0], color[1], color[2]);
        }
    }
    return colors;
}


Vector3Array PointCloudProcessor::extractNormalArray(const cv::Mat& normal_image, const cv::Mat& depth_image) const
{
    Vector3Array normals;
    for (int v=0; v<normal_image.rows; ++v)
    {
        for (int u=0; u<normal_image.cols; ++u)
        {
            if (depth_image.at<DepthImageType>(v, u) <=0.0) continue;
            const NormalImageType& normal = normal_image.at<NormalImageType>(v, u);
            normals.emplace_back(normal[0], normal[1], normal[2]);
        }
    }
    return normals;
}

}