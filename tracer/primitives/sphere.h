#pragma once

#include "../object.h"

namespace tracer {

/**
 * Equation: normal_.dot(p) + distance_ = 0;
 **/
class Sphere : public ObjectMinimal<float> {
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
	
	
	std::optional<float> trace_minimal(const Ray& ray) const noexcept override {
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
			if (ray.in_range(d1))      { Eigen::Vector3f p = ray.at(d1); return Hit(d1,p,(p-center()).normalized()); }
			else if (ray.in_range(d2)) { Eigen::Vector3f p = ray.at(d2); return Hit(d2,p,(p-center()).normalized()); }
			else return {};
		}
		float d = (-ray.origin().dot(normal()) - distance())/ray.direction().dot(normal());
		if (ray.in_range(d)) return d; 
		else return {};
	}
	
	float minimal_distance(const float& minhit) const noexcept override {
		return minhit;
	}
	
	Hit hit_from_minimal(const Ray& ray, const float& d) const noexcept override{
		return Hit(d, ray.at(d), normal());
	}
	std::optional<Hit> trace(const Ray& ray) const noexcept override {
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
			if (ray.in_range(d1))      { Eigen::Vector3f p = ray.at(d1); return Hit(d1,p,(p-center()).normalized()); }
			else if (ray.in_range(d2)) { Eigen::Vector3f p = ray.at(d2); return Hit(d2,p,(p-center()).normalized()); }
			else return {};
		}
	}
};

};
