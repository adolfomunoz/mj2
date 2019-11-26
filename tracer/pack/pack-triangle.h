#pragma once

#include "pack.h"
#include "../primitives/triangle.h"
#include <array>

namespace tracer {

template<int N> //Make it ObjectGeneral<std::tuple<float, int>>
class Pack<Triangle,N> : public ObjectImpl<Pack<Triangle,N>> {
	Eigen::Matrix<float,N,3> geometric_normals_;
	Eigen::Matrix<float,N,3> geometric_normals1_;
	Eigen::Matrix<float,N,3> geometric_normals2_;
	Eigen::Matrix<float,N,1> distances_;
	Eigen::Matrix<float,N,1> distances1_;
	Eigen::Matrix<float,N,1> distances2_;
	std::array<Triangle,N> triangles_;

	//We focus the efficency on the trace method, not in the construction of the structure which obviously is rather slow.
	template<typename Collection>
	void setup(const Collection& c) {
		assert(c.size() <= N);
		int n = 0;
		for (const Triangle& t : c) {
			geometric_normals_.row(n) = (t.point1() - t.point0()).cross(t.point2() - t.point0());
			geometric_normals1_.row(n) = (t.point2() - t.point0()).cross(geometric_normals_.row(n))/geometric_normals_.row(n).squaredNorm();
			geometric_normals2_.row(n) = geometric_normals_.row(n).cross(t.point1() - t.point0())/geometric_normals_.row(n).squaredNorm();
			distances_(n) = -geometric_normals_.row(n).dot(t.point0());
			distances1_(n) = -geometric_normals1_.row(n).dot(t.point0());
			distances2_(n) = -geometric_normals2_.row(n).dot(t.point0());
			triangles_[n] = t;
			++n;	
		}
		--n;
		//We copy the last plane to fill the whole pack (should not happen very often...)
		for (int i = (n+1); i<N; ++i) {
			geometric_normals_.row(i) = geometric_normals_.row(n);
			geometric_normals1_.row(i) = geometric_normals1_.row(n);
			geometric_normals2_.row(i) = geometric_normals2_.row(n);
			distances_(i) = distances_(n);
			distances1_(i) = distances1_(n);
			distances2_(i) = distances2_(n);
			triangles_[i] = triangles_[n];
		}
	}	
public:
	Pack(const std::list<Triangle>& c)      { setup(c); }
	Pack(const std::vector<Triangle>& c)    { setup(c); }

	const Eigen::Matrix<float,N,3>& geometric_normals() const noexcept { return geometric_normals_; }
	const Eigen::Matrix<float,N,3>& geometric_normals1() const noexcept { return geometric_normals1_; }
	const Eigen::Matrix<float,N,3>& geometric_normals2() const noexcept { return geometric_normals2_; }
	const Eigen::Matrix<float,N,1>& distances() const noexcept { return distances_; }
	const Eigen::Matrix<float,N,1>& distances1() const noexcept { return distances1_; }
	const Eigen::Matrix<float,N,1>& distances2() const noexcept { return distances2_; }
	const std::array<Triangle,N>& triangles() const noexcept { return triangles_; }

	
/** 
  * Using the Möller-Trumbore intersection algorithm:
  * https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  * Warning: it might not be as efficient (early rejection do not apply). We did not manage to make it work and the other is more parallel.
	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		const float eps = 1.e-6f;
		Eigen::Matrix<float,N,3> h = -(edge2s().colwise().cross(ray.direction())); //for all edge2 cross product with ray direction
		Eigen::Matrix<float,N,1> a = edge1s().cwiseProduct(h).rowwise().sum();  //Element wise dot product
//		if ((a > -eps) && (a < eps)) return {};

		Eigen::Matrix<float,N,1> f = a.cwiseInverse();
		Eigen::Matrix<float,N,3> s = -1.0f*(point0s().rowwise() - ray.origin().transpose());
		Eigen::Matrix<float,N,1> u = f.cwiseProduct(s.cwiseProduct(h).rowwise().sum());  //Element wise dot product

//		if ((u < 0.0f) || (u > 1.0f)) return {};

		Eigen::Matrix<float,N,3> q;
		//Element wise cross product. Sadly, we need a loop.
		for (int i = 0; i<N; ++i) q.row(i) = s.row(i).cross(edge1s().row(i));       
		Eigen::Matrix<float,N,1> v = f.cwiseProduct(q*ray.direction()); //for all q dot product with ray.direction()
//		if ((v < 0.0f) || (u + v > 1.0f)) return {};

		Eigen::Matrix<float,N,1> t = f.cwiseProduct(q.cwiseProduct(edge2s()).rowwise().sum()); //Element wise dot product


		std::optional<Hit> hit; Ray r = ray;
		int n = -1;
		for (int i = 0; i<N; ++i) {
			if ((abs(a[i]) >= eps) && (u[i] >= 0.0f) && (v[i] >= 0.0f) && ((u[i]+v[i]) <= 1.0f) && (r.in_range(t[i]))) {
				n = i;
				r.set_range_max(t[i]);
			}
		}

		if (n < 0) return { };
		else       return triangles()[n].hit(ray,std::make_tuple(t[n],u[n],v[n]));
	}
*/

/** 
  * Using the Havel-Herout algorithm:
  * http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5159346
  **/
	template <typename T> 
	static int sgn(T val) {
    		return (T(0) < val) - (val < T(0));
	}


	std::optional<std::tuple<float,float,float,const Triangle*>> trace_general(const Ray& ray) const noexcept {
		const float eps = 1.e-6f;
		Eigen::Matrix<float,N,1> det = geometric_normals()*ray.direction();
//		if ((det > -eps) && (det < eps)) return {}; //Its parallel
		Eigen::Matrix<float,N,1> taux = -distances() - geometric_normals()*ray.origin();
		Eigen::Matrix<float,N,1> t    = det.cwiseInverse().cwiseProduct(taux);
		Eigen::Matrix<float,N,3> paux = det*ray.origin().transpose() + taux*ray.direction().transpose();
		Eigen::Matrix<float,N,1> uaux = paux.cwiseProduct(geometric_normals1()).rowwise().sum() + distances1().cwiseProduct(det);
		Eigen::Matrix<float,N,1> vaux = paux.cwiseProduct(geometric_normals2()).rowwise().sum() + distances2().cwiseProduct(det);
//		const float uaux = paux.dot(geometric_normal1()) + det*distance1();
//		if (sgn(uaux) != sgn(det - uaux)) return {}; //Out of range u
//		const float vaux = paux.dot(geometric_normal2()) + det*distance2();
//		if (sgn(vaux) != sgn(det - uaux - vaux)) return {}; //Out of range v

		Ray r = ray; int n = -1;
		for (int i = 0; i<N; ++i) {
			if ( (abs(det(i))>eps) && r.in_range(t(i)) && (sgn(uaux(i))==sgn(det(i) - uaux(i))) && (sgn(vaux(i))==sgn(det(i) - uaux(i) - vaux(i))) ) {
				n = i;
				r.set_range_max(t(i));
			}
		}

		if (n < 0) return {};
		else return std::tuple<float,float,float,const Triangle*>(t(n), uaux(n)/det(n), vaux(n)/det(n), &triangles_[n]); 
	}
	
	Hit hit(const Ray& ray, const std::tuple<float, float, float, const Triangle*>& h) const noexcept {
		return std::get<3>(h)->hit(ray,
			std::tuple<float,float,float>(std::get<0>(h),std::get<1>(h),std::get<2>(h)));
	}

};


}
