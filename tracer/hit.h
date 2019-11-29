#pragma once

#include <Eigen/Dense>
#include <memory>

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
			assert(std::abs(normal.squaredNorm() - 1.0)<1.e-3);  //normal should be normalized
			assert(std::abs(tangent.squaredNorm() - 1.0)<1.e-3); //tangent should be normalized
			assert(std::abs(normal.dot(tangent))<1.e-3);   //normal and tangent should be perpendicular
			local_to_global_.col(0) = tangent;
			local_to_global_.col(1) = normal.cross(tangent);
			local_to_global_.col(2) = normal;
		}

	//It is not a good idea to rely on this constructor very much
	Hit(float distance, const Eigen::Vector3f& point, const Eigen::Vector3f& normal) noexcept :
		Hit(distance, point, normal, Eigen::Vector3f(normal[1],normal[2],normal[0]).cross(normal).normalized()) { }
		
	constexpr float distance() const noexcept { return distance_; }
	const Eigen::Vector3f& point() const noexcept { return point_; }
	const Eigen::Matrix3f& local_to_global() const noexcept { return local_to_global_; }
	
	auto tangent() const noexcept { return local_to_global().col(0); }
	auto bitangent() const noexcept { return local_to_global().col(1); }	
	auto normal() const noexcept { return local_to_global().col(2); }
//	constexpr const Plane& object() const noexcept { return object_; }
	
#ifdef MATERIAL
    std::shared_ptr<MATERIAL> mat;
public:
    const std::shared_ptr<MATERIAL>& material() const { return mat; }
    Hit& set_material(const std::shared_ptr<MATERIAL>& m) { 
        if (m) mat = m;
        return (*this); 
    }
#endif
};

};
