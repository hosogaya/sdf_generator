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

    struct CommonConfig
    {
        int width_;
        int height_;
        Scalar min_d_;
        Scalar min_z_;
        Scalar depth_smooth_thres_ratio_;
        bool is_loop_; // for yaw angle. The others are assumed not to be loop.
    };

    PointCloudProcessor(const CommonConfig& common_config);
    ~PointCloudProcessor();

    /**
     * @param points: the coordinate of the nearest point in each direction.
     * @param colors: the color of the nearest point in each direction. 
     * @param nomarls: the normal vector towards the sensor to the surface in each direction. 
     */
    void process(PointArray& points, ColorArray& colors, Vector3Array& normals);

protected:
    void projectPointCloudToImage(
        const PointArray& point, const ColorArray& color, 
        cv::Mat& vertex_map, cv::Mat& depth_image, 
        cv::Mat& color_image, Scalar min_d, Scalar min_z);

    virtual Scalar projectPointToImage(const Point& point, int& u, int& v) = 0;


    virtual cv::Mat calNormalImage(const cv::Mat& vertex_image, const cv::Mat& depth_image);
    virtual PointArray extractPointArray(const cv::Mat& vertex_image, const cv::Mat& depth_image) const;
    virtual ColorArray extractColorArray(const cv::Mat& color_image, const cv::Mat& depth_image) const;
    virtual Vector3Array extractNormalArray(const cv::Mat& normal_image, const cv::Mat& depth_image) const;

    const CommonConfig common_config_;
};

}