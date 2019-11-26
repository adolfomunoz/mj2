#pragma once

#include "pack.h"
#include "../primitives/axis-aligned-box.h"
#include <array>
#include <tuple>

namespace tracer {

template<int N>
class Pack<AxisAlignedBox,N> : public ObjectImpl<Pack<AxisAlignedBox,N>> {
	Eigen::Array<float,N,3> mins_;
	Eigen::Array<float,N,3> maxs_;
	std::array<AxisAlignedBox,N> boxes_;

	//We focus the efficency on the trace method, not in the construction of the structure which obviously is rather slow.
	template<typename Collection>
	void setup(const Collection& c) {
		assert(c.size() <= N);
		int n = 0;
		for (const AxisAlignedBox& b : c) {
			mins_.row(n) = b.min();
			maxs_.row(n) = b.max();
			boxes_[n] = b;
			++n;	
		}
		--n;
		//We copy the last plane to fill the whole pack (should not happen very often...)
		for (int i = (n+1); i<N; ++i) {
			mins_.row(i) = mins_.row(n);
			maxs_.row(i) = maxs_.row(n);
			boxes_[i] = boxes_[n];
		}
	}	
public:
	Pack(const std::list<AxisAlignedBox>& c)      { setup(c); }
	Pack(const std::vector<AxisAlignedBox>& c)    { setup(c); }
	
	const Eigen::Array<float,N,3>& mins() const noexcept { return mins_; }
	const Eigen::Array<float,N,3>& maxs() const noexcept { return maxs_; }
	const std::array<Plane,N>& boxes() const noexcept { return boxes_; }

    static AABoxRay ray_type(const Ray& r) noexcept 
	{ return AABoxRay(r); }

	std::optional<std::tuple<float,int>> trace_general(const AABoxRay& ray) const noexcept {
		Eigen::Array<float,N,3> t1 = (mins().rowwise() - ray.origin().transpose().array()).rowwise()*ray.inv_direction().transpose().array();
		Eigen::Array<float,N,3> t2 = (maxs().rowwise() - ray.origin().transpose().array()).rowwise()*ray.inv_direction().transpose().array();
		Eigen::Array<float,N,1> tmin = t1.cwiseMin(t2).rowwise().maxCoeff();
		Eigen::Array<float,N,1> tmax = t1.cwiseMax(t2).rowwise().minCoeff();

/**		if ((!Eigen::isfinite(t1).all()) || (!Eigen::isfinite(t2).all())) {
			std::cout<<t1<<std::endl<<std::endl<<t2<<std::endl<<std::endl<<t1.cwiseMin(t2)<<std::endl<<std::endl<<t1.cwiseMax(t2)<<std::endl<<std::endl<<tmin<<std::endl<<std::endl<<tmax<<std::endl<<"--------------"<<std::endl;
		}
**/

		std::optional<std::tuple<float,int>> hit; Ray r = ray;
		for (int i = 0; i<N; ++i) {
			if (tmin[i] <= tmax[i]) {
				if (r.in_range(tmin[i])) { hit = std::make_tuple(tmin[i],i); r.set_range_max(tmin[i]); }
				else if (r.in_range(tmax[i])) { hit = std::make_tuple(tmax[i],i); r.set_range_max(tmax[i]); }
			};
		}
		return hit;
	}	

	//This is not supposed to be efficient. Very often (BVH) we're needing just the floating point number
	Hit hit(const AABoxRay& ray, const std::tuple<float, int>& t) const noexcept {
		Eigen::Vector3f p = ray.at(std::get<0>(t));
		Eigen::Vector3f n(0.0f,0.0f,0.0f);

		for (int i = 0; i<3; ++i) {
			if (fabs(p[i]-mins()(std::get<1>(t),i))<1.e-6f)      { n[i] = -1.0f;  return Hit(std::get<0>(t),p,n); }
			else if (fabs(p[i]-maxs()(std::get<1>(t),i))<1.e-6f) { n[i] =  1.0f;  return Hit(std::get<0>(t),p,n); }
		}
		
		return Hit(std::get<0>(t), p, n);
	}

};


}
