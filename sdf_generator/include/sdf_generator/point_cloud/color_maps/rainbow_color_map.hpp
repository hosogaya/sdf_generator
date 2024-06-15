#pragma once

#include <sdf_generator/point_cloud/color_maps/color_map.hpp>

namespace sdf_generator
{
class RainbowColorMap: public ColorMap
{
public:
    virtual Color colorLookup(Scalar value) const
    {
        Scalar v = std::min(max_value_, std::max(value, min_value_));
        v = (v - min_value_)/ (max_value_ - min_value_);

        return Color::convert2Rainbow(v);
    }
};
}