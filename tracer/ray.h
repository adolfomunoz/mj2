#pragma once

#include <Eigen/Dense>
#include <tuple>

namespace tracer {
	
class Ray {
	Eigen::Vector3f origin_;
	Eigen::Vector3f direction_;
	std::tuple<float, float> range_;

public:
	Ray(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, float range_min = 0.0f, float range_max = std::numeric_limits<float>::infinity()) noexcept:
		origin_(origin), direction_(direction), range_(range_min, range_max)
		{}
	
	const Eigen::Vector3f& origin() const noexcept    { return origin_;    }
	const Eigen::Vector3f& direction() const noexcept { return direction_; }
	const std::tuple<float, float>& range() const noexcept { return range_; }	
	float range_min() const noexcept { return std::get<0>(range()); }
	float range_max() const noexcept { return std::get<1>(range()); }
	
	void set_range_min(float value) noexcept { std::get<0>(range_)=value; }
	void set_range_max(float value) noexcept { std::get<1>(range_)=value; }
	
	bool in_range(float distance) const noexcept {
		return std::isfinite(distance)&&(range_min()<=distance)&&(distance<=range_max());
	}
	
	Eigen::Vector3f at(float distance) const noexcept { return origin()+distance*direction(); }
};

};
