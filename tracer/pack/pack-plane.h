#pragma once

#include "pack.h"
#include "../primitives/plane.h"
#include <array>

namespace tracer {

template<int N>
class Pack<Plane,N> : public Object {
	Eigen::Matrix<float,N,3> normals_;
	Eigen::Matrix<float,N,1> distances_;
	std::array<Plane,N> planes_;

	//We focus the efficency on the trace method, not in the construction of the structure which obviously is rather slow.
	template<typename Collection>
	void setup(const Collection& c) {
		assert(c.size() <= N);
		int n = 0;
		for (const Plane& p : c) {
			normals_.row(n) = p.normal();
			distances_[n] = p.distance();
			planes_[n] = p;
			++n;	
		}
		//We copy the last plane to fill the whole pack (should not happen very often...)
		for (int i = n; i<N; ++i) {
			normals_.row(i) = normals_.row(n);
			distances_[i] = distances_[n];
			planes_[i] = planes_[n];
		}
	}	
public:
	Pack(const std::list<Plane>& c)      { setup(c); }
	Pack(const std::vector<Plane>& c)    { setup(c); }
	
	const Eigen::Matrix<float,N,3>& normals() const noexcept { return normals_; }
	const Eigen::Matrix<float,N,1>& distances() const noexcept { return distances_; }
	const std::array<Plane,N>& planes() const noexcept { return planes_; }

	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		Eigen::Matrix<float,N,1> d = -(normals() * ray.direction()).cwiseInverse().cwiseProduct(normals() * ray.origin() + distances());

		std::optional<Hit> hit; Ray r = ray;
		for (int i = 0; i<N; ++i) {
			if (r.in_range(d[i])) {
				hit = Hit(d[i]);
				r.set_range_max(d[i]);
			}
		}
		return hit;
	}	
};


}
