#pragma once

#include <Eigen/Dense>

namespace tracer {
class Hit {
	float distance_;
	Eigen::Vector3f point_;
	Eigen::Matrix3f local_to_global_;
//	const Plane& object_;
public:
	Hit(float distance, const Eigen::Vector3f& point, const Eigen::Vector3f& normal, const Eigen::Vector3f& tangent) noexcept :
		distance_(distance), point_(point) 
		{
			assert(std::abs(normal.norm2() - 1.0)<1.e-5);  //normal should be normalized
			assert(std::abs(tangent.norm2() - 1.0)<1.e-5); //tangent should be normalized
			assert(std::abs(normal.dot(tangent))<1.e-5);   //normal and tangent should be perpendicular
			local_to_global_.col(0) = tangent;
			local_to_global_.col(1) = normal.cross(tangent);
			local_to_global_.col(2) = normal;
		}

	//It is not a good idea to rely on this constructor very much
	Hit(float distance, const Eigen::Vector3f& point, const Eigen::Vector3f& normal) noexcept :
		Hit(distance, point, normal, Eigen::Vector3f(0,1,0).cross(normal).normalized()) { }
		
	float distance() const noexcept { return distance_; }
	const Eigen::Vector3f& point() const noexcept { return point_; }
	const Eigen::Matrix3f& local_to_global() const noexcept { return local_to_global_; }
	
	auto tangent() const noexcept { return local_to_global().col(0); }
	auto bitangent() const noexcept { return local_to_global().col(1); }	
	auto normal() const noexcept { return local_to_global().col(2); }
//	constexpr const Plane& object() const noexcept { return object_; }
	
};

};
