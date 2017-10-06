#pragma once

#include "../object.h"
#include <tuple>

namespace tracer {

/**
 * Using the Möller-Trumbore intersection algorithm. 
 *
 * The "HitType" is (t,u,v) where t is the ray parameter, and u and v are coordinates on the surface
 * of the triangle.
 **/
class Triangle : public GeneralObject<std::tuple<float,float,float>> {
	Eigen::Vector3f edge1_, edge2_;
	Eigen::Vector3f point0_, point1_, point2_;
	Eigen::Vector3f normal0_, normal1_, normal2_;
	Eigen::Vector3f tangent0_, tangent1_, tangent2_;
public:
	Triangle(const Eigen::Vector3f& point0, const Eigen::Vector3f& normal0, const Eigen::Vector3f& tangent0,
		 const Eigen::Vector3f& point1, const Eigen::Vector3f& normal1, const Eigen::Vector3f& tangent1,
		 const Eigen::Vector3f& point2, const Eigen::Vector3f& normal2, const Eigen::Vector3f& tangent2) :
		edge1_(point1 - point0), edge2_(point2 - point0),
		point0_(point0), point1_(point1), point2_(point2),
		normal0_(normal0), normal1_(normal1), normal2_(normal2),
		tangent0_(tangent0), tangent1_(tangent1), tangent2_(tangent2)  
	{
		assert(std::abs(normal0_.norm2() - 1.0)<1.e-5);  //normal should be normalized
		assert(std::abs(normal1_.norm2() - 1.0)<1.e-5);  //normal should be normalized
		assert(std::abs(normal2_.norm2() - 1.0)<1.e-5);  //normal should be normalized
		assert(std::abs(tangent0_.norm2() - 1.0)<1.e-5); //tangent should be normalized
		assert(std::abs(tangent1_.norm2() - 1.0)<1.e-5); //tangent should be normalized
		assert(std::abs(tangent2_.norm2() - 1.0)<1.e-5); //tangent should be normalized
	}

	Triangle(const Eigen::Vector3f& point0, const Eigen::Vector3f& normal0,
		 const Eigen::Vector3f& point1, const Eigen::Vector3f& normal1,
		 const Eigen::Vector3f& point2, const Eigen::Vector3f& normal2) :
		Triangle(point0, normal0, (point1-point0).normalized(),
			 point1, normal1, (point1-point0).normalized(),
			 point2, normal2, (point1-point0).normalized()) 
	{ }

	Triangle(const Eigen::Vector3f& point0,
		 const Eigen::Vector3f& point1,
		 const Eigen::Vector3f& point2, const Eigen::Vector3f& normal) :
		Triangle(point0, normal, point1, normal, point2, normal) { }

	Triangle(const Eigen::Vector3f& point0,
		 const Eigen::Vector3f& point1,
		 const Eigen::Vector3f& point2) :
		Triangle(point0, 
			 point1, 
			 point2, (point1-point0).cross(point2-point0).normalized()) 
	{ }


	const Eigen::Vector3f& point0() const noexcept { return point0_; }
	const Eigen::Vector3f& point1() const noexcept { return point1_; }
	const Eigen::Vector3f& point2() const noexcept { return point2_; }
	const Eigen::Vector3f& normal0() const noexcept { return normal0_; }
	const Eigen::Vector3f& normal1() const noexcept { return normal1_; }
	const Eigen::Vector3f& normal2() const noexcept { return normal2_; }
	const Eigen::Vector3f& tangent0() const noexcept { return tangent0_; }
	const Eigen::Vector3f& tangent1() const noexcept { return tangent1_; }
	const Eigen::Vector3f& tangent2() const noexcept { return tangent2_; }
	const Eigen::Vector3f& edge1() const noexcept { return edge1_; }
	const Eigen::Vector3f& edge2() const noexcept { return edge2_; }

	float hit_distance(const std::tuple<float, float, float>& h) const noexcept override {
		return std::get<0>(h);
	}

 /** 
  * Using the Möller-Trumbore intersection algorithm:
  * https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  **/
	std::optional<std::tuple<float,float,float>> trace_general(const Ray& ray) const noexcept override {
		const float eps = 1.e-6f;
		Eigen::Vector3f h = ray.direction().cross(edge2());
		float a = edge1().dot(h);
		if ((a > -eps) && (a < eps)) return {};

		float f = 1.0f/a;
		Eigen::Vector3f s = ray.origin() - point0();
		float u = f*(s.dot(h));

		if ((u < 0.0f) || (u > 1.0f)) return {};

		Eigen::Vector3f q = s.cross(edge1());
		float v = f*ray.direction().dot(q);
		if ((v < 0.0f) || (u + v > 1.0f)) return {};

		float t = f*edge2().dot(q);

		if (ray.in_range(t)) return std::make_tuple(t,u,v);
		else return {};
	}
	
	Hit hit(const Ray& ray, const std::tuple<float, float, float>& h) const noexcept override {
		float t, u, v;
		std::tie(t,u,v) = h;
		return Hit(t, ray.at(t), 
			((1.0f - u - v)*normal0() + u*normal1() + v*normal2()).normalized(),
			((1.0f - u - v)*tangent0() + u*tangent1() + v*tangent2()).normalized());
	}
};

};
