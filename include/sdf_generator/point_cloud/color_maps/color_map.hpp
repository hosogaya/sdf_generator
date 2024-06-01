#pragma once

#include <algorithm>
#include <sdf_generator/point_cloud/type.hpp>

namespace sdf_generator
{
class ColorMap
{
public:
    using Ptr = std::shared_ptr<ColorMap>;

    ColorMap(): min_value_(0.0), max_value_(1.0) {}
    ~ColorMap() {}

    void setMinValue(Scalar value) {min_value_ = value;}
    void setMaxValue(Scalar value) {max_value_ = value;}

    virtual Color colorLookup(Scalar value) const = 0;

protected:
    Scalar min_value_;
    Scalar max_value_;
};

}