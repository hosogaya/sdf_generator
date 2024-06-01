#pragma once

#include <sdf_generator/point_cloud/type.hpp>

namespace sdf_generator
{
class PointCloudProcessor
{
public: 
    using VertexImageType = cv::Vec3f;
    using DepthImageType = Scalar;
    using ColorImageType = cv::Vec3b;
    using NormalImageType = cv::Vec3f;

    PointCloudProcessor(const int width, const int height, const Scalar min_d, const Scalar min_z, const Scalar depth_smooth_thres_ratio, const bool is_loop);
    ~PointCloudProcessor();

    void process(PointArray& point, ColorArray& color, Vector3Array& normal);

    void projectPointCloudToImage(
        const PointArray& point, const ColorArray& color, 
        cv::Mat& vertex_map, cv::Mat& depth_image, 
        cv::Mat& color_image, Scalar min_d, Scalar min_z);

    virtual Scalar projectPointToImage(const Point& point, int& u, int& v) = 0;


    virtual cv::Mat calNormalImage(const cv::Mat& vertex_image, const cv::Mat& depth_image);
    virtual PointArray extractPointArray(const cv::Mat& vertex_image, const cv::Mat& depth_image) const;
    virtual ColorArray extractColorArray(const cv::Mat& color_image, const cv::Mat& depth_image) const;
    virtual Vector3Array extractNormalArray(const cv::Mat& normal_image, const cv::Mat& depth_image) const;

protected:
    const int width_;
    const int height_;
    const bool is_loop_;

    const Scalar depth_smooth_thres_ratio_;
    const Scalar min_z_;
    const Scalar min_d_;
};

}