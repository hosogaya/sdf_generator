#pragma once

#include <sdf_generator/core/common.hpp>

namespace sdf_generator
{
struct TsdfVoxel
{
    Scalar distance_ = 0.0;
    Scalar weight_ = 0.0;
    Ray gradient_ = Ray::Zero();
    bool occupied_ = false;
    bool isBehind() const {return distance_ < 0.0;}
};

}