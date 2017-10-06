#pragma once

#include "pack.h"
#include "../primitives/sphere.h"
#include <array>

namespace tracer {

template<int N> //Make it ObjectGeneral<std::tuple<float, int>>
class Pack<Sphere,N> : public Object {
	Eigen::Matrix<float,N,3> centers_;
	Eigen::Matrix<float,N,1> radiuses2_;
	std::array<Sphere,N> spheres_;

	//We focus the efficency on the trace method, not in the construction of the structure which obviously is rather slow.
	template<typename Collection>
	void setup(const Collection& c) {
		assert(c.size() <= N);
		int n = 0;
		for (const Sphere& s : c) {
			centers_.row(n) = s.center();
			radiuses2_[n] = s.radius2();
			spheres_[n] = s;
			++n;	
		}
		//We copy the last plane to fill the whole pack (should not happen very often...)
		for (int i = n; i<N; ++i) {
			centers_.row(i) = centers_.row(n);
			radiuses2_[i] = radiuses2_[n];
			spheres_[i] = spheres_[n];
		}
	}	
public:
	Pack(const std::list<Sphere>& c)      { setup(c); }
	Pack(const std::vector<Sphere>& c)    { setup(c); }
	
	const Eigen::Matrix<float,N,3>& centers() const noexcept { return centers_; }
	const Eigen::Matrix<float,N,1>& radiuses2() const noexcept { return radiuses2_; }
	const std::array<Sphere,N>& spheres() const noexcept { return spheres_; }

	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		Eigen::Matrix<float,N,3> oc = centers().rowwise() - ray.origin().transpose();
		float a = ray.direction().squaredNorm();
		Eigen::Matrix<float,N,1> b = -2.0f*(oc*ray.direction());
		Eigen::Matrix<float,N,1> c = oc.rowwise().squaredNorm() - radiuses2();

		Eigen::Matrix<float,N,1> disc = b.cwiseProduct(b) - 4.0f*a*c;
		
		Eigen::Matrix<float,N,1> sqrtdisc = disc.cwiseSqrt();
		float inv2a = 0.5f/a;
		Eigen::Matrix<float,N,1> d1 = (-b - sqrtdisc)*inv2a;
		Eigen::Matrix<float,N,1> d2 = (-b + sqrtdisc)*inv2a;

		std::optional<Hit> hit; Ray r = ray;
	        int n = -1;	
		for (int i = 0; i<N; ++i) {
			if (disc[i]>0) {
				if (r.in_range(d1[i])) {
					n = i;
					r.set_range_max(d1[i]);
				}
				else if (r.in_range(d2[i])) {
					n = i;
					r.set_range_max(d2[i]);
				}
			}
		} 		
		
		if (n<0) return { };
		else {
		   // r.range_max() holds the final distance
		   Eigen::Vector3f p = ray.at(r.range_max());    
		   return Hit(r.range_max(),p,(p - centers().row(n).transpose()).normalized());
		}
	}	
};


}
