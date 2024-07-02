#include <sdf_generator/integrator/tsdf_integrator.hpp>

namespace sdf_generator
{
TsdfIntegratorBase::TsdfIntegratorBase(const Config& config, Layer<TsdfVoxel>::Ptr layer)
: config_(config)
{
    setLayer(layer);

    if (config.integrator_threads_ == 0)
    {
        config_.integrator_threads_ = 1;
    }
    if (config_.allow_clear_ && !config_.voxel_carving_enabled_)
    {
        config_.allow_clear_ = false;
    }
}

TsdfIntegratorBase::~TsdfIntegratorBase() {}

void TsdfIntegratorBase::setLayer(Layer<TsdfVoxel>::Ptr layer)
{
    layer_ = layer;

    voxel_size_ = layer_->voxelSize();
    block_size_ = layer_->blockSize();
    voxels_per_side_ = layer_->voxelsPerSide();

    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

TsdfVoxel* TsdfIntegratorBase::allocateStorageAndGetVoxelPtr(const GlobalIndex& voxel_index, Block<TsdfVoxel>::Ptr last_block, BlockIndex& last_block_index)
{
    const BlockIndex block_index = calBlockIndex(voxel_index, voxels_per_side_inv_);
    if ((block_index != last_block_index) || !last_block)
    {
        last_block = layer_->getBlockPtr(block_index);
        last_block_index = block_index;
    }

    if (!last_block)
    {
        std::lock_guard<std::mutex> lock(temp_block_mutex_);

        typename Layer<TsdfVoxel>::BlockHashMap::iterator itr = temp_block_map_.find(block_index);
        if (itr != temp_block_map_.end())
        {
            last_block = itr->second;
        }
        else
        {
            auto insert_status = temp_block_map_.emplace(
                block_index, std::make_shared<Block<TsdfVoxel>>(
                    voxels_per_side_, voxel_size_,
                    calOrigin(block_index, block_size_)
                )
            );

            last_block = insert_status.first->second;
        }
    }

    last_block->setUpdatedAll();

    const VoxelIndex local_voxel_index = calLocalVoxelIndex(voxel_index, voxels_per_side_);

    return &(last_block->getVoxel(local_voxel_index));   
}

void TsdfIntegratorBase::updateLayerWithStoredBlocks() 
{
    for (const std::pair<const BlockIndex, Block<TsdfVoxel>::Ptr>& block_pair: temp_block_map_)
    {
        layer_->insertBlock(block_pair);
    }

    temp_block_map_.clear();
}


void TsdfIntegratorBase::updateTsdfVoxel(
    const TransformMatrix<Scalar>& tf_global2current, const Point& origin, 
    const Point& point_c, const Point& point_g,
    const Vector3& normal_c, const Vector3& normal_g,
    const GlobalIndex& global_voxel_index, const Color& color,
    const Scalar init_weight, TsdfVoxel& tsdf_voxel)
{
    const Point voxel_center = calCenterPoint(global_voxel_index, voxel_size_);
    Scalar distance = calDistance(origin, point_g, voxel_center);
    
    std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_index));

    bool in_the_reliable_band = true;
    if (distance > config_.default_truncation_distance_*config_.reliable_band_ratio_)
        in_the_reliable_band = false;

    Vector3 gradient_c;
    if (config_.normal_available_ && in_the_reliable_band)
    {
        Scalar normal_ratio(1.0);
        if (tsdf_voxel.gradient_.norm() > kWeightEpsilon)
        {
            gradient_c = tf_global2current.rotation().conjugate()*tsdf_voxel.gradient_;
            // Condition 1. curve surface
            if (config_.curve_assumption_ && normal_c.norm() > kWeightEpsilon)
            {
                Scalar cos_theta = std::abs(gradient_c.dot(point_c)) / point_c.norm();
                Scalar cos_alpha = std::abs(gradient_c.dot(normal_c)) / normal_c.norm();
                Scalar sin_theta = std::sqrt(1.0f - cos_theta*cos_theta);
                Scalar sin_alpha = std::sqrt(1.0f - cos_alpha*cos_alpha);
                normal_ratio = std::abs(sin_theta*(cos_alpha-1)/sin_alpha + cos_theta);

                if (std::isnan(normal_ratio)) normal_ratio = cos_theta;
            }
            // Condition 2. flat surface
            else
            {
                normal_ratio = std::abs(gradient_c.dot(point_c)) / point_c.norm();
            }
        }
        else if (normal_c.norm() > kWeightEpsilon)
        {
            // gradient not ready yet
            normal_ratio = std::abs(normal_c.dot(point_c)) / point_c.norm();
        }

        if (normal_ratio < config_.reliable_normal_ratio_thre_) 
        {
            std::cout << "[updateTsdfVoxel] normal ratio is too smalll: " << normal_ratio << std::endl;
            return;
        }
        // get the non-projective sdf, if it's still larger than truncation
        // distance, the gradient would not be updated
        distance *= normal_ratio;
    }

    if (distance < -config_.default_truncation_distance_) 
    {
        std::cout << "[updateTsdfVoxel] the distance is too small: " << distance << std::endl;
        return;
    }
    bool with_init_weight(false);
    if (init_weight > 0.0f) with_init_weight = true;

    Scalar weight =calVoxelWeight(point_c, distance, with_init_weight, init_weight);

    if (weight < kWeightEpsilon) 
    {
        std::cout << "[updateTsdfVoxel] the weight is too small: " << weight << std::endl;
        return; 
    }
    if (distance > config_.default_truncation_distance_)
    {
        updateTsdfVoxelValue(tsdf_voxel, distance, weight);
        tsdf_voxel.distance_ = std::min(config_.default_truncation_distance_, tsdf_voxel.distance_);
    }
    else if (config_.normal_available_)
    {
        if (normal_g.norm() > kWeightEpsilon)  
            updateTsdfVoxelGradient(tsdf_voxel, normal_g, weight);

        updateTsdfVoxelValue(tsdf_voxel, distance, weight, &color);
    }

    // std::cout << "[updateTsdfVoxel] The distance: " << tsdf_voxel.distance_ << ", The weight: " << tsdf_voxel.weight_ << std::endl;
}

void TsdfIntegratorBase::updateTsdfVoxelValue(
    TsdfVoxel& voxel, const Scalar distance, const Scalar weight,
    const Color* color) const
{
    Scalar new_weight = voxel.weight_ + weight;
    if (new_weight < kWeightEpsilon) 
    {
        std::cout << "[updateTsdfVoxelValue] the new weight is too small: " << new_weight << std::endl;
        return;
    }
    voxel.distance_ = (voxel.distance_*voxel.weight_ + distance*weight) / new_weight;

    voxel.weight_ = std::min(new_weight, config_.max_weight_);

    if (color != nullptr)
        voxel.color_ = Color::blendTwoColors(voxel.color_, voxel.weight_, *color, weight);
}

void TsdfIntegratorBase::updateTsdfVoxelGradient(
    TsdfVoxel& voxel, const Vector3 normal, const Scalar weight) const
{
    Scalar new_weight = voxel.weight_ + weight;
    if (new_weight < kWeightEpsilon) 
    {
        std::cout << "[updateTsdfVoxelGradient] the new weight is too small: " << new_weight << std::endl;
        return;
    }
    if (voxel.gradient_.norm() > kWeightEpsilon)
        voxel.gradient_ = (voxel.gradient_*voxel.weight_ + normal*weight).normalized();
    else
        voxel.gradient_ = normal.normalized();
}

Scalar TsdfIntegratorBase::calDistance(
    const Point& origin, const Point& point_g,
    const Point& voxel_center) const
{
    const Vector3 origin2voxel = voxel_center - origin;
    const Vector3 origin2point = point_g - origin;

    const Scalar dist_point = origin2point.norm();
    const Scalar dist_g_voxel = origin2voxel.dot(origin2point)/dist_point;

    return dist_point - dist_g_voxel;
}


Scalar TsdfIntegratorBase::calVoxelWeight(
    const Point& point_c, const Scalar distance, const bool with_init_weight,
    const Scalar init_weight) const
{
    Scalar weight(1.0);
    if (with_init_weight) weight = init_weight;
    // Part 1. Weight reduction with distance
    else if (!config_.use_const_weight_) weight /= std::pow(point_c.norm(), config_.weight_reduction_exp_);

    // Part 2. weight drop-off
    if (config_.use_weight_dropoff_)
    {
        const Scalar dropoff_epsilon = config_.weight_dropoff_epsilon_ > 0.0f
                                      ? config_.weight_dropoff_epsilon_
                                      : config_.weight_dropoff_epsilon_*-voxel_size_;
        if (distance < -dropoff_epsilon)
        {
            weight *= (config_.default_truncation_distance_ + distance)
                    / (config_.default_truncation_distance_ - dropoff_epsilon);
            weight = std::max(weight, 0.0f);
        }
    }

    // Part 3. deal with sparse point cloud
    if (config_.use_sparsity_compensation_factor_)
    {
        if (std::abs(distance) < config_.default_truncation_distance_)
            weight *= config_.sparsity_compensation_factor_;
    }

    return weight;
}

Scalar TsdfIntegratorBase::getVoxelWeight(const Point& point_c) const
{
    if (config_.use_const_weight_) return 1.0f;

    const Scalar dist = point_c.norm();
    if (dist > kCoordinateEpsilon) return 1.0f/std::pow(dist, config_.weight_reduction_exp_);

    return 0.0f;
}

}