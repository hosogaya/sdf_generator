#include <sdf_generator/core/esdf_map.hpp>

namespace sdf_generator
{
bool EsdfMap::getDistanceAtPosition(
    const Eigen::Vector3d& position, double& distance
) const
{
    constexpr bool interpolate = true;
    return getDistanceAtPosition(position, distance, interpolate);
}

bool EsdfMap::getDistanceAtPosition(
    const Eigen::Vector3d& position, double& distance, bool interpolate
) const
{
    Scalar distance_temp;
    bool success = interpolator_.getDistance(
        position.cast<Scalar>(), distance_temp, interpolate
    );
    if (success) distance = static_cast<double>(distance_temp);

    return success;
}

bool EsdfMap::getDistanceAndGradientAtPosition(
        const Eigen::Vector3d& position, double& distance, 
        Eigen::Vector3d& gradient
) const
{
    constexpr bool interpolate = true;
    return getDistanceAndGradientAtPosition(
        position, distance, gradient, interpolate
    );
}

bool EsdfMap::getDistanceAndGradientAtPosition(
    const Eigen::Vector3d& position, double& distance, 
    Eigen::Vector3d& gradient, bool interpolate
) const
{
    Scalar distance_fp;
    Vector3 gradient_fp;
    bool success;
    success = interpolator_.getDistance(
        position.cast<Scalar>(), distance_fp, interpolate
    );
    success &= interpolator_.getGradient(
        position.cast<Scalar>(), gradient_fp, interpolate
    );

    distance = static_cast<double>(distance_fp);
    gradient = gradient_fp.cast<double>();

    return success;
}

bool EsdfMap::isObserved(const Eigen::Vector3d& position) const
{
    Point point = position.cast<Scalar>();
    typename Block<EsdfVoxel>::ConstPtr block_ptr = esdf_layer_->getBlockConstPtr(point);
    if (!block_ptr) return false;

    const EsdfVoxel& voxel = block_ptr->getConstVoxel(point);
    return voxel.observed_;
}

void EsdfMap::batchGetDistanceAtPosition(
    EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions, 
    Eigen::Ref<Eigen::VectorXd> distances, 
    Eigen::Ref<Eigen::VectorXi> observed
) const
{
    if (distances.size() < positions.cols()) 
        throw std::runtime_error("Distances array smaller than number of queries");
    
    if (observed.size() < positions.cols())
        throw std::runtime_error("Observed array smaller than number of queries");
    
    for (int i=0; i<positions.cols(); ++i)
        observed[i] = getDistanceAtPosition(positions.col(i), distances[i]);
}

void EsdfMap::batchGetDisntaceAndGradientAtPosition(
    EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions, 
    Eigen::Ref<Eigen::VectorXd> distances, 
    EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& gradients, 
    Eigen::Ref<Eigen::VectorXi> observed
)
{
     if (distances.size() < positions.cols()) 
        throw std::runtime_error("Distances array smaller than number of queries");
    
    if (observed.size() < positions.cols())
        throw std::runtime_error("Observed array smaller than number of queries");
    
    if (gradients.size() < positions.cols())
        throw std::runtime_error("Gradients array smaller than number of queries");

    for (int i=0; i<positions.cols(); ++i)
    {
        Eigen::Vector3d grad;
        observed[i] = getDistanceAndGradientAtPosition(
            positions.col(i), distances[i], grad
        );

        gradients.col(i) = grad;
    }
}

void EsdfMap::batchIsObserved(
    EigenDRef<const Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions, 
    Eigen::Ref<Eigen::VectorXi> observed
) const
{
    if (observed.size() < positions.cols())
        throw std::runtime_error("Observed array smaller than number of queries");

    for (int i=0; i<positions.cols(); ++i)
        observed[i] = isObserved(positions.col(i));
}


unsigned int EsdfMap::coordPlaneSliceGetDistance(
    unsigned int free_plane_index, double free_plane_val,
    EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,  
    Eigen::Ref<Eigen::VectorXd> distances, unsigned int max_points
) const
{
    BlockIndexList blocks;
    esdf_layer_->getAllAllocatedBlocks(blocks);

    // cache layer settings
    size_t vps = esdf_layer_->voxelsPerSide();
    size_t num_voxels_per_block = vps*vps*vps;

    bool did_all_fit = true;
    unsigned int count = 0;

    for (const auto& index : blocks)
    {
        const Block<EsdfVoxel>::ConstPtr block_ptr = esdf_layer_->getBlockConstPtr(index);

        const Point& origin = block_ptr->origin();
        if (std::abs(origin(free_plane_index) - free_plane_val)
         > block_ptr->blockSize()) 
        {
            continue;
        }

        for (size_t linear_index=0; linear_index<num_voxels_per_block; ++linear_index)
        {
            Point coord = block_ptr->calCoordinate(linear_index);
            const EsdfVoxel& voxel = block_ptr->getConstVoxel(linear_index);
            if (std::abs(coord(free_plane_index) - free_plane_val) <= block_ptr->voxelSize())
            {
                double distance;
                if (voxel.observed_) distance = voxel.distance_;
                else continue;

                if (count < positions.cols())
                    positions.col(count) = Eigen::Vector3d(coord.x(), coord.y(), coord.z());
                else did_all_fit = false;

                if (count < distances.size())
                    distances(count) = distance;
                else did_all_fit = false;

                ++count;
                if (count >= max_points) return count;
            }
        }
    }

    if (!did_all_fit)
    {
        throw std::runtime_error(
            std::string("Unable to store ") + std::to_string(count) + "values. "
        );
    }

    return count;
}
}