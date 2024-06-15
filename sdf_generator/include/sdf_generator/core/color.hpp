#pragma once

#include <sdf_generator/core/type.hpp>

namespace sdf_generator
{
struct Color
{
    using Value = uint8_t;

    Color(): r_(0), g_(0), b_(0), a_(0) {}
    Color(Value r, Value g, Value b, Value a)
    : r_(r), g_(g), b_(b), a_(a) {}
    Color(Value r, Value g, Value b)
    : Color(r, g, b, 0) {}

    Value r_;
    Value g_;
    Value b_;
    Value a_;

    static Color blendTwoColors(
        const Color& first, Scalar first_weight, 
        const Color& second, Scalar second_weight
    )
    {
        Scalar total_weight = first_weight + second_weight;
        first_weight /= total_weight;
        second_weight /= total_weight;

        Color res;
        res.r_ = static_cast<Value>(first.r_*first_weight + second.r_*second_weight);
        res.g_ = static_cast<Value>(first.g_*first_weight + second.g_*second_weight);
        res.b_ = static_cast<Value>(first.b_*first_weight + second.b_*second_weight);
        res.a_ = static_cast<Value>(first.a_*first_weight + second.a_*second_weight);

        return res;
    }


    // color samples
    static const Color white()
    {
        return Color(255,255, 255);
    }
    static const Color Black()
    {
        return Color(0, 0, 0);
    }
    static const Color Red() 
    {
        return Color(255, 0, 0);
    }
    static const Color Green()
    {
        return Color(0, 255, 0);
    }
    static const Color Blue()
    {
        return Color(0, 0, 255);
    }
    static const Color Yellow()
    {
        return Color(255, 255, 0);
    }
    static const Color Orange() {
        return Color(255, 127, 0);
    }
    static const Color Purple() {
        return Color(127, 0, 255);
    }
    static const Color Teal() {
        return Color(0, 255, 255);
    }
    static const Color Pink() {
        return Color(255, 0, 127);
    }

    static inline Color convert2Gray(Scalar value) 
    {
        Color color;
        color.a_ = 255;
        color.r_ = round(value*255);
        color.g_ = color.r_;
        color.b_ = color.r_;

        return color;
    }
    static inline Color convert2Rainbow(Scalar value)
    {
        Color color;
        color.a_ = 255;
        
        Scalar s = 1.0;
        Scalar v = 1.0;

        value -= std::floor(value);
        value += 6.0;

        int i;
        Scalar m, n, f;

        i = std::floor(value);
        f = value - i;
        if (!(i & 1))
            f = 1 - f;  // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
            color.r_ = 255 * v;
            color.g_ = 255 * n;
            color.b_ = 255 * m;
            break;
            case 1:
            color.r_ = 255 * n;
            color.g_ = 255 * v;
            color.b_ = 255 * m;
            break;
            case 2:
            color.r_ = 255 * m;
            color.g_ = 255 * v;
            color.b_ = 255 * n;
            break;
            case 3:
            color.r_ = 255 * m;
            color.g_ = 255 * n;
            color.b_ = 255 * v;
            break;
            case 4:
            color.r_ = 255 * n;
            color.g_ = 255 * m;
            color.b_ = 255 * v;
            break;
            case 5:
            color.r_ = 255 * v;
            color.g_ = 255 * m;
            color.b_ = 255 * n;
            break;
            default:
            color.r_ = 255;
            color.g_ = 127;
            color.b_ = 127;
            break;
        }

        return color;
    }
};
    
using ColorArray = AlignedVector<Color>;

}