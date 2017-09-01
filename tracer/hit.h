#pragma once

#include <Eigen/Dense>

namespace tracer {
class Hit {
	float distance_;
	Eigen::Vector3f point_;
	Eigen::Vector3f normal_;
//	const Plane& object_;
public:
	Hit(float distance, const Eigen::Vector3f& point, const Eigen::Vector3f& normal) noexcept :
		distance_(distance), point_(point), normal_(normal) { }
		
	float distance() const noexcept { return distance_; }
	const Eigen::Vector3f& point() const noexcept { return point_; }
	const Eigen::Vector3f& normal() const noexcept { return normal_; }
//	constexpr const Plane& object() const noexcept { return object_; }
	
};

};
