#pragma once

#include "../object.h"

namespace tracer {

class Sphere : public ObjectImpl<Sphere> { //GeneralObject<float> {
	Eigen::Vector3f center_;
	float radius_;  
	float radius2_;

public:
	Sphere(const Eigen::Vector3f& center, float radius) noexcept:
		center_(center), radius_(radius), radius2_(radius*radius) { }
	Sphere() noexcept:
		Sphere(Eigen::Vector3f(0.0,0.0,0.0f),1.0f) { }
		
	const Eigen::Vector3f& center() const noexcept { return center_; }
	float radius() const noexcept { return radius_; }
	float radius2() const noexcept { return radius2_; }
	
	float implicit_function(const Eigen::Vector3f& point) const noexcept {
		return (point - center()).squaredNorm() - radius2();
	}
	
	
	std::optional<float> trace_general(const Ray& ray) const noexcept {
		Eigen::Vector3f oc = ray.origin() - center();
		float a = ray.direction().squaredNorm();
		float b = 2.0f*ray.direction().dot(oc);
		float c = oc.squaredNorm() - radius2();

		float disc = b*b - 4*a*c;
		if (disc < 0) return {};
		else {
			float sqrtdisc = std::sqrt(disc);
			float inv2a = 0.5f/a;
			float d1 = (-b - sqrtdisc)*inv2a;
			float d2 = (-b + sqrtdisc)*inv2a;
			if (ray.in_range(d1))      return d1; 			
			else if (ray.in_range(d2)) return d2;
			else return {};
		}
	}
	
	Hit hit(const Ray& ray, float d) const noexcept {
		Eigen::Vector3f p = ray.at(d);
		return Hit(d, p, (p - center()).normalized());
	}
};

};
