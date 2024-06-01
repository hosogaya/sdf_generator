#pragma once

#include <sdf_generator/core/type.hpp>
#include <Eigen/Core>

namespace sdf_generator
{
template <typename Scalar>
class TransformMatrix
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TransformMatrix() {trans_.setZero(); quat_.setIdentity();}
    TransformMatrix(Eigen::Vector3<Scalar>& trans, Eigen::Quaternion<Scalar>& quat)
    : trans_(trans), quat_(quat) {}
    ~TransformMatrix() {}

    inline TransformMatrix operator*(const TransformMatrix& other) const noexcept
    {
        TransformMatrix temp;
        temp.quat_ = quat_*other.quat_;
        temp.trans_ = trans_ + quat_*other.trans_;
        return temp;
    }

    inline auto operator*(const Eigen::Vector3<Scalar_t>& other) const noexcept
    {
        return trans_ + quat_*other;
    }

    inline auto operator*(const Eigen::Quaternion<Scalar_t>& other) const noexcept
    {
        return quat_*other;
    }

    inline auto operator*(const Jacobian<Scalar_t>& other) const noexcept
    {
        return quat_*(other.colwise() + trans_);
    }

    inline void setZero() {
        quat_.setZero();
        trans_.setZero();
    }

    inline void setIdentity()
    {
        quat_.setIdentity();
        trans_.setZero();
    }

    inline TransformMatrix inverse() const
    {
        return TransformMatrix(quat_.transpose(), -quat_.transpose()*trans_);
    }

    inline void setRotationMatrix(const Scalar_t& v, const Eigen::Vector3<Scalar_t>& axis)
    {
        quat_ = AngleAxis<Scalar_t>(v, axis).toRotationMatrix();
    }

private:
    Eigen::Vector3<Scalar> trans_;
    Eigen::Quaternion<Scalar> quat_;
};
}