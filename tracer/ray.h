#pragma once

#include <Eigen/Dense>
#include <tuple>

namespace tracer {
	
class Ray {
	Eigen::Vector3f origin_;
	Eigen::Vector3f direction_;
	std::tuple<float, float> range_;

public:
	Ray(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, 
		const std::tuple<float, float>& range = std::make_tuple(0.0f, std::numeric_limits<float>::infinity())) noexcept:
		origin_(origin), direction_(direction), range_(range)
		{}
	
	constexpr const Eigen::Vector3f& origin() const noexcept    { return origin_;    }
	constexpr const Eigen::Vector3f& direction() const noexcept { return direction_; }
	constexpr const std::tuple<float, float>& range() const noexcept { return range_; }	
	constexpr float range_min() const noexcept { return std::get<0>(range()); }
	constexpr float range_max() const noexcept { return std::get<1>(range()); }
	
	void set_range_min(float value) noexcept { std::get<0>(range_)=value; }
	void set_range_max(float value) noexcept { std::get<1>(range_)=value; }
	
	constexpr bool in_range(float distance) const noexcept {
		return std::isfinite(distance)&&(range_min()<=distance)&&(distance<=range_max());
	}
	
	Eigen::Vector3f at(float distance) const noexcept { return origin()+distance*direction(); }
};

};