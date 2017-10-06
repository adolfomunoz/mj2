#pragma once

#include "../object.h"

namespace tracer {

/**
 * Equation: normal_.dot(p) + distance_ = 0;
 **/
class Plane : public GeneralObject<float> {
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
	
	std::optional<float> trace_general(const Ray& ray) const noexcept override {
		float d = (-ray.origin().dot(normal()) - distance())/ray.direction().dot(normal());
		if (ray.in_range(d)) return d; 
		else return {};
	}
	
	Hit hit(const Ray& ray, float d) const noexcept override {
		return Hit(d, ray.at(d), normal());
	}
};

};
