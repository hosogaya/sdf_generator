#include <sdf_generator/integrator/thread_safe_index.hpp>

namespace sdf_generator
{
ThreadSafeIndex::ThreadSafeIndex(size_t number_of_points)
    : atomic_idx_(0), number_of_points_(number_of_points) 
{}

bool ThreadSafeIndex::getNextIndex(size_t& idx) {
    size_t sequential_idx = atomic_idx_.fetch_add(1);
	
    if (sequential_idx >= number_of_points_) {
		return false;
	} else {
		idx = getNextIndexImpl(sequential_idx);
		return true;
	}
}

void ThreadSafeIndex::reset() {
  	atomic_idx_.store(0);
}

// simple thread safe index
SimpleThreadSafeIndex::SimpleThreadSafeIndex(size_t number_of_points)
: ThreadSafeIndex(number_of_points) {}

size_t SimpleThreadSafeIndex::getNextIndexImpl(size_t base_idx)
{
    return base_idx;
}

// mixed thread safe index
MixedThreadSafeIndex::MixedThreadSafeIndex(size_t number_of_points)
    : ThreadSafeIndex(number_of_points),
      number_of_groups_(number_of_points / step_size_) 
{}

size_t MixedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx) 
{
    if (number_of_groups_ * step_size_ <= sequential_idx) {
        return sequential_idx;
    }

    const size_t group_num = sequential_idx % number_of_groups_;
    const size_t position_in_group = sequential_idx / number_of_groups_;

    return group_num * step_size_ + position_in_group;
}


// stored thread safe index
SortedThreadSafeIndex::SortedThreadSafeIndex(const PointArray& points_c)
    : ThreadSafeIndex(points_c.size()) 
{
    indices_and_squared_norms_.reserve(points_c.size());
    size_t idx = 0;
    for (const Point& point_c : points_c) {
        indices_and_squared_norms_.emplace_back(idx, point_c.squaredNorm());
        ++idx;
    }

    std::sort(
        indices_and_squared_norms_.begin(), indices_and_squared_norms_.end(),
        [](const std::pair<size_t, double>& a,
            const std::pair<size_t, double>& b) { return a.second < b.second; });
}

size_t SortedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx) {
    return indices_and_squared_norms_[sequential_idx].first;
}

ThreadSafeIndex* ThreadSafeIndexFactory::get(const std::string& mode, const PointArray& points_c)
{
    if (mode == "simple")
        return new SimpleThreadSafeIndex(points_c.size());
  	else if (mode == "mixed")
		return new MixedThreadSafeIndex(points_c.size());
	else if (mode == "sorted")
		return new SortedThreadSafeIndex(points_c);
	else
		return nullptr;
}

}