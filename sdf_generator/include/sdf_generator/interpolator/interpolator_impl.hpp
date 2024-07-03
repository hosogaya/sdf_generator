#include <sdf_generator/interpolator/util.hpp>

namespace sdf_generator
{
template <typename VoxelType>
bool Interpolator<VoxelType>::getDistance(const Point& pos, Scalar& distance, bool interpolate) const
{
    if (interpolate)
    {
        return getInterpDistance(pos, distance);
    }
    else
    {
        return getNearestDistance(pos, distance);
    }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getWeight(const Point& pos, Scalar& weight, bool interpolate) const
{
    if (interpolate)
    {
        return getInterpWeight(pos, weight);
    }
    else 
    {
        return getNearestWeight(pos, weight);
    }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxel(const Point& pos,  VoxelType& voxel, bool interpolate) const
{
    if (interpolate)
    {
        return getInterpVoxel(pos, voxel);
    }
    else 
    {
        return getNearestVoxel(pos, voxel);
    }
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getGradient(const Point& pos, Vector3& grad, bool interpolate) const
{
    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockPtr(pos);
    if (block_ptr == nullptr) return false;

    grad = Vector3::Zero();
    // Iterate over all 3D, and over negative and positive signs in central difference
    for (int i=0; i<3; ++i)
    {
        for (int sign = -1; sign<2; ++sign)
        {
            Point offset = Point::Zero();
            offset(i) = sign*block_ptr->voxelSize();
            Scalar offset_distance;
            if (!getDistance(pos + offset, offset_distance, interpolate)) return false;

            grad(i) += offset_distance*static_cast<Scalar>(sign);
        }
    }

    // Scale by correct size
    // This is central difference, so it's 2x voxel size between measurements.
    grad /= (2*block_ptr->voxelSize());
    return true;
}


template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpDistance(const Point& pos, Scalar& distance) const
{
    const VoxelType* voxels[8];
    InterpVector q_vector;
    if (!getVoxelsAndQVector(pos, voxels, q_vector))
    {
        return false;
    }
    else
    {
        distance = interpMember(q_vector, voxels, &getVoxelDistance);
    }
    return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpWeight(const Point& pos, Scalar& weight) const
{
    const VoxelType* voxels[8];
    InterpVector q_vector;
    if (!getVoxelsAndQVector(pos, voxels, q_vector)) return false;

    weight = interpMember(q_vector, voxels, &getVoxelWeight);

    return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getInterpVoxel(const Point& pos, VoxelType& voxel) const
{
    const VoxelType* voxels[8];
    InterpVector q_vector;
    if (!getVoxelsAndQVector(pos, voxels, q_vector)) return false;

    voxel = interpVoxel(q_vector, voxels);
    return true;
}


template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestDistance(const Point& pos, Scalar& distance) const
{
    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockConstPtr(pos);
    if (block_ptr == nullptr) return false;

    const VoxelType& voxel = block_ptr->getConstVoxel(pos);

    distance = getVoxelDistance(voxel);

    return isObservedVoxel(voxel);
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestWeight(const Point& pos, Scalar& weight) const
{
    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockConstPtr(pos);
    if (block_ptr == nullptr) return false;

    const VoxelType& voxel = block_ptr->getConstVoxel(pos);

    weight = getVoxelWeight(voxel);

    return isObservedVoxel(voxel);
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestVoxel(const Point& pos, VoxelType& voxel) const
{
    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockConstPtr(pos);
    if (block_ptr == nullptr) return false;

    voxel = block_ptr->getVoxel(pos);

    return isObservedVoxel(voxel);
}


template <typename VoxelType>
bool Interpolator<VoxelType>::setIndexes(const Point& pos, BlockIndex& block_index, InterpIndexes& voxel_indexes) const
{
    block_index = layer_->getBlockIndex(pos);

    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockConstPtr(block_index);
    if (block_ptr == nullptr) return false;

    VoxelIndex voxel_index = block_ptr->calVoxelIndex(pos);

    // shift index to bottom left corner voxel
    Point center_offset = pos - block_ptr->calCoordinate(voxel_index);

    for (int i=0; i<center_offset.rows(); ++i)
    {
        // ensure that the ponit we are interpolating to is always larger than 
        // the center of the voxel_index in all dimenssions
        if (center_offset(i) < 0.0)
        {
            --voxel_index(i);
            // move blocks if needed
            if (voxel_index(i) < 0)
            {
                --block_index(i);
                voxel_index(i) += block_ptr->voxelsPerSide();
            }
        }
    }

    // get indexes of neighbors

    // FROM PAPER (http://spie.org/samples/PM159.pdf)
    // clang-format off
    voxel_indexes <<
        0, 0, 0, 0, 1, 1, 1, 1,
        0, 0, 1, 1, 0, 0, 1, 1,
        0, 1, 0, 1, 0, 1, 0, 1;
    // clang-format on

    voxel_indexes.colwise() += voxel_index.array();
    return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxelsAndQVector(
    const BlockIndex& block_index, const InterpIndexes& voxel_indexes,
    const Point& pos, const VoxelType** voxels, InterpVector& q_vector) const
{
    for (int i=0; i<voxel_indexes.cols(); ++i)
    {
        typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockConstPtr(block_index);
        if (block_ptr == nullptr) return false;

        VoxelIndex voxel_index = voxel_indexes.col(i);
        // if voxel index is too large get neighboring block and update index
        if ((voxel_index.array() >= block_ptr->voxelsPerSide()).any())
        {
            BlockIndex new_block_index = block_index;
            for (int j=0; j<block_index.rows(); ++j)
            {
                if (voxel_index(j) > static_cast<IndexElement>(block_ptr->voxelsPerSide()))
                {
                    new_block_index(j)++;
                    voxel_index(j) -= block_ptr->voxelsPerSide();
                }
            }
            block_ptr = layer_->getBlockPtr(new_block_index);
            if (block_ptr == nullptr) return false;
        }

        // use bottom left corner voxel to compute weights vector
        if (i == 0) getQVector(block_ptr->calCoordinate(voxel_index), pos, block_ptr->voxelSizeInv(), &q_vector);
    
        const VoxelType& voxel = block_ptr->getConstVoxel(voxel_index);

        voxels[i] = & voxel;
        if (isObservedVoxel(voxel))
        {
            return false;
        }
    }
    return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getVoxelsAndQVector(const Point& pos, const VoxelType** voxels, InterpVector& q_vector) const
{
    BlockIndex block_index;
    InterpIndexes voxel_indexes;
    if (!setIndexes(pos, block_index, voxel_indexes)) return false;

    return getVoxelsAndQVector(block_index, voxel_indexes, pos, voxels, q_vector);
}

template <typename VoxelType>
void Interpolator<VoxelType>::getQVector(const Point& voxel_pos, const Point& pos, const Scalar voxel_size_inv, InterpVector* q_vector) const
{
    const Point voxel_offset = (pos - voxel_pos)*voxel_size_inv;

    // FROM PAPER (http://spie.org/samples/PM159.pdf)
    // clang-format off
    *q_vector <<
        1,
        voxel_offset[0],
        voxel_offset[1],
        voxel_offset[2],
        voxel_offset[0] * voxel_offset[1],
        voxel_offset[1] * voxel_offset[2],
        voxel_offset[2] * voxel_offset[0],
        voxel_offset[0] * voxel_offset[1] * voxel_offset[2];
    // clang-format on
}

template <typename VoxelType>
template <typename Getter_t>
inline Scalar Interpolator<VoxelType>::interpMember(const InterpVector& q_vector, const VoxelType** voxels, Getter_t (*getter)(const VoxelType&))
{
    InterpVector data;
    for (int i=0; i<data.size(); ++i)
    {
        data[i] = static_cast<Scalar>((*getter)(*voxels[i]));
    }

    // FROM PAPER (http://spie.org/samples/PM159.pdf)
    // clang-format off
    static const InterpTable interp_table =
      (InterpTable() <<
        1,  0,  0,  0,  0,  0,  0,  0,
       -1,  0,  0,  0,  1,  0,  0,  0,
       -1,  0,  1,  0,  0,  0,  0,  0,
       -1,  1,  0,  0,  0,  0,  0,  0,
        1,  0, -1,  0, -1,  0,  1,  0,
        1, -1, -1,  1,  0,  0,  0,  0,
        1, -1,  0,  0, -1,  1,  0,  0,
       -1,  1,  1, -1,  1, -1, -1,  1
       ).finished();

    // clang-format on
    return q_vector * (interp_table * data.transpose());
}

template <>
inline Scalar Interpolator<TsdfVoxel>::getVoxelDistance(const TsdfVoxel& voxel)
{
    return voxel.distance_;
}

template <>
inline Scalar Interpolator<EsdfVoxel>::getVoxelDistance(const EsdfVoxel& voxel)
{
    return voxel.distance_;
}

template <typename VoxelType>
inline Scalar Interpolator<VoxelType>::getVoxelWeight(const VoxelType& voxel)
{
    return 0.0;
}

template <>
inline Scalar Interpolator<TsdfVoxel>::getVoxelWeight(const TsdfVoxel& voxel)
{
    return voxel.weight_;
}

template <>
inline Scalar Interpolator<EsdfVoxel>::getVoxelWeight(const EsdfVoxel& voxel)
{
    return voxel.observed_ ? 1.0 : 0.0;
}

template <>
inline Color::Value Interpolator<TsdfVoxel>::getRed(const TsdfVoxel& voxel)
{
    return voxel.color_.r_;
}

template <>
inline Color::Value Interpolator<TsdfVoxel>::getGreen(const TsdfVoxel& voxel)
{
    return voxel.color_.g_;
}

template <>
inline Color::Value Interpolator<TsdfVoxel>::getBlue(const TsdfVoxel& voxel)
{
    return voxel.color_.b_;
}

template <>
inline Color::Value Interpolator<TsdfVoxel>::getAlpha(const TsdfVoxel& voxel)
{
    return voxel.color_.a_;
}

template <>
inline TsdfVoxel Interpolator<TsdfVoxel>::interpVoxel(const InterpVector& q_vector, const TsdfVoxel** voxels)
{
    TsdfVoxel voxel;
    voxel.distance_ = interpMember(q_vector, voxels, &getVoxelDistance);
    voxel.weight_ = interpMember(q_vector, voxels, &getVoxelWeight);
    voxel.color_.r_ = interpMember(q_vector, voxels, &getRed);
    voxel.color_.g_ = interpMember(q_vector, voxels, &getGreen);
    voxel.color_.b_ = interpMember(q_vector, voxels, &getBlue);
    voxel.color_.a_ = interpMember(q_vector, voxels, &getAlpha);

    return voxel;
}


template <typename VoxelType>
bool Interpolator<VoxelType>::getAdaptiveDistanceAndGradient(
    const Point& point, Scalar& distance, Vector3& grad
) const
{
    Scalar nearest_neighbor_distance = 0.0f;
    if (!getDistance(point, nearest_neighbor_distance, false))
        return false;

    // Then try to get the interpolated distance
    bool has_interpolated_distance = getDistance(point, distance, true);

    // Now try to estimate the gradient. Same general procedure as getGradient()
    // above, but also allow finite difference methods other than central
    // difference (left difference, right difference).
    typename Layer<VoxelType>::BlockType::CosntPtr block_ptr = layer_->getBlockConstPtr(point);
    if (block_ptr == nullptr) return false;

    Vector3 gradient(0.0f, 0.0f, 0.0f);

    // Try to get the full gradient if possible
    bool has_interpolated_grad = false;
    if (has_interpolated_distance)
        has_interpolated_grad = getGradient(point, gradient, true);

    if (!has_interpolated_grad)
    {
        // Otherwise fall back to this
        for (unsigned int i=0; i<3; ++i)
        {
            // First Check if we can get both sides for central difference
            Vector3 offset(0.0f, 0.0f, 0.0f);
            offset(i) = block_ptr->voxelSize();
            Scalar left_distance = 0.0f;
            Scalar right_distance = 0.0f;
            bool left_valid = getDistance(point - offset, left_distance, false);
            bool right_valid = getDistance(point + offset, right_distance, false);

            if (left_valid && right_valid)
            {
                gradient(i) = (right_distance - left_distance) 
                             *(0.5f*block_ptr->voxelSizeInv());
            }
            else if (left_valid)
            {
                gradient(i) = (nearest_neighbor_distance - left_distance) 
                             *(block_ptr->voxelSizeInv()); 
            }
            else if (right_valid)
            {
                gradient(i) = (right_distance - nearest_neighbor_distance)
                             *(block_ptr->voxelSizeInv());
            }
            else
            {
                return false;
            }
        }
    }

    if (!has_interpolated_distance)
    {
        VoxelIndex voxel_index = block_ptr->calTruncatedVoxelIndex(point);
        Point voxel_pos = block_ptr->calCoordinate(voxel_index);

        Vector3 voxel_offset = point - voxel_pos;
        distance = nearest_neighbor_distance + voxel_offset.dot(gradient);
    }

    grad = gradient;
    return true;
}

template <typename VoxelType>
bool Interpolator<VoxelType>::getNearestDistanceAndWeight(
    const Point& point, Scalar& distance, Scalar& weight
) const
{
    typename Layer<VoxelType>::BlockType::ConstPtr block_ptr = layer_->getBlockConstPtr(point);
    if (block_ptr == nullptr) return false;

    const VoxelType& voxel = block_ptr->getVoxel(point);
    distance = getVoxelDistance(voxel);
    weight = getVoxelWeight(voxel);

    return true;
}

}