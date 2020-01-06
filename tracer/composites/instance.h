#pragma once

#include "../object.h"

namespace tracer {

class Instance : public ObjectImpl<Instance> {
	Eigen::Affine3f transform;
	Eigen::Affine3f inverse;
	Object object;
	
public:
	std::optional<Hit> trace_general(const Ray& r) const noexcept {
		auto h = object.trace(Ray(inverse*r.origin(),inverse.linear()*r.direction(),r.range_min(),r.range_max()));
		if (h) {
			return Hit(h->distance(),transform * h->point(), 
				(inverse.linear().transpose()*h->normal()).normalized(), 
				(inverse.linear().transpose()*h->tangent()).normalized())
				#ifdef MATERIAL
				.set_material(this->material())
				#endif
			;
		}
		return h;
	}
	bool trace_shadow(const Ray& r) const noexcept override { 
		return object.trace_shadow(Ray(inverse*r.origin(),inverse.linear()*r.direction(),r.range_min(),r.range_max()));
	}
	
	//O should be convertible to Object
	template<typename T, typename O>
	Instance(T&& t, O&& o) :
		transform(std::forward<T>(t)), inverse(t.inverse()), object(std::forward<O>(o)) {}
};

}