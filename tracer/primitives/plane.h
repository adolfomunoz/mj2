#pragma once

#include "../object.h"

namespace tracer {

/**
 * Equation: normal_.dot(p) + distance_ = 0;
 **/
class Plane : public ObjectImpl<Plane> { //GeneralObject<float> {
	Eigen::Vector3f normal_;
	float distance_;  

public:
	Plane(const Eigen::Vector3f& normal, float distance) noexcept:
		normal_(normal.normalized()), distance_(distance) { }
	Plane() noexcept:
		Plane(Eigen::Vector3f(0.0,0.0,-1.0f),0.0f) { }
		
	const Eigen::Vector3f& normal() const noexcept { return normal_; }
	float distance() const noexcept { return distance_; }
	
	Plane(const Eigen::Vector3f& normal, const Eigen::Vector3f& point) :
		Plane(normal, -normal.normalized().dot(point)) {}
		
	float implicit_function(const Eigen::Vector3f& point) const noexcept {
		return normal().dot(point) + distance();
	}
	
	std::optional<float> trace_general(const Ray& ray) const noexcept {
		std::optional<float> sol;
		float den = ray.direction().dot(normal());
		if (std::abs(den) < 1.e-10) return sol;
		float d = (-ray.origin().dot(normal()) - distance())/den;
		if (ray.in_range(d)) return sol = d;
		return sol;
	}
	
	Hit hit(const Ray& ray, float d) const noexcept {
		return Hit(d, ray.at(d), normal());
	}
};

};
