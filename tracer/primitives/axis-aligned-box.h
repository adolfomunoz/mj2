#pragma once

#include "../object.h"
#include <limits>

namespace tracer {

class AABoxRay : public Ray {
	Eigen::Vector3f inv_direction_;
public:
	AABoxRay(const Ray& r) noexcept : Ray(r), inv_direction_(r.direction().cwiseInverse()) { }
	const Eigen::Vector3f& inv_direction() const noexcept { return inv_direction_; } 
};


class AxisAlignedBox : public ObjectImpl<AxisAlignedBox> { //GeneralObject<float,AABoxRay> {
	Eigen::Vector3f min_, max_;

public:
	AxisAlignedBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max) noexcept :
		min_(min.cwiseMin(max)), max_(min.cwiseMax(max)) { }

	AxisAlignedBox() noexcept:
		min_(Eigen::Vector3f(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max())),
		max_(Eigen::Vector3f(std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest()))
		{ }

	const Eigen::Vector3f& min() const noexcept { return min_; }
	const Eigen::Vector3f& max() const noexcept { return max_; }
	
    static AABoxRay extend_ray(const Ray& r) noexcept { return AABoxRay(r); }

	std::optional<float> trace_general(const AABoxRay& ray) const noexcept {
		std::optional<float> sol;
		Eigen::Vector3f t1 = (min() - ray.origin()).cwiseProduct(ray.inv_direction());
		Eigen::Vector3f t2 = (max() - ray.origin()).cwiseProduct(ray.inv_direction());
//		std::cerr<<min().transpose()<<"\t|\t"<<max().transpose()<<std::endl;
//		std::cerr<<ray.inv_direction().transpose()<<std::endl;
//		std::cerr<<t1.transpose()<<"\t|\t"<<t2.transpose()<<std::endl;

		float tmin = t1.cwiseMin(t2).maxCoeff();
		float tmax = t1.cwiseMax(t2).minCoeff();
//		std::cerr<<tmin<<" - "<<tmax<<std::endl<<std::endl;

		if (tmax >= tmin) {
			if (ray.in_range(tmin)) 	 sol = tmin;
			else if (ray.in_range(tmax)) sol = tmax;
		}
		return sol;
	}

	//This is not supposed to be efficient. Very often (BVH) we're needing just the floating point number
	Hit hit(const AABoxRay& ray, float d) const noexcept {
		Eigen::Vector3f p = ray.at(d);
		Eigen::Vector3f n(0.0f,0.0f,0.0f);

		for (int i = 0; i<3; ++i) {
			if (fabs(p[i]-min()[i])<1.e-6f)      { n[i] = -1.0f;  return Hit(d,p,n); }
			else if (fabs(p[i]-max()[i])<1.e-6f) { n[i] =  1.0f;  return Hit(d,p,n); }
		}
		
		return Hit(d, p, n);
	}
};

};
