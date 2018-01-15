#pragma once

#include <optional>
#include "ray.h"
#include "hit.h"

namespace tracer {

class Object {
public:
        Ray ray_type(const Ray& r) const noexcept { return r; }                  //So it still can be called externally.
        const Hit& hit(const Ray& r, const Hit& h) const noexcept { return h; }  //So it still can be called externally.
        float hit_distance(const Hit& h) const noexcept { return h.distance(); } //So it still can be called externally.

	virtual std::optional<Hit> trace(const Ray& r) const noexcept = 0;
	virtual bool trace_shadow(const Ray& r) const { return bool(trace(r)); }

	std::optional<Hit> trace_general(const Ray& r) const noexcept { return trace(r); } //So it still can be called externally.
};


/**
 * RayType is the type that defines a ray that should intersect the object. It is often Ray, but it is useful that it becomes something different for optimizations 
 *    (specially for lists or packs). The object itself can get a RayType from a Ray when needed. RayType should inherit from ray, or at least have the same methods.
 * 
 * HitType is the type that defines the minimal information returned by the object when intersected with a ray. It can be Hit, but it is useful if it is something
 *    smaller that is significative enough to deduce intersections. The object can transform it into a full ray.
 **/
template<typename HitType, typename RayType = Ray>
class GeneralObject : public Object {
public: 
	virtual RayType                ray_type(const Ray& r) const noexcept = 0; //Should return the same thing for all the objects and do not depend on any local parameter.
	virtual Hit                    hit(const RayType& r, const HitType& h) const noexcept = 0;
	virtual float                  hit_distance(const HitType& h) const noexcept = 0; //Should return the same thing for all the objects and do not depend on any local parameter (although it is not enforced)
	virtual std::optional<HitType> trace_general(const RayType& r) const noexcept = 0;

	std::optional<Hit> trace(const Ray& r) const noexcept override {
		RayType rt = this->ray_type(r);
		std::optional<HitType> h = trace_general(rt);
		if (h) return this->hit(rt, *h); else return {};
	}
	virtual bool trace_shadow(const Ray& r) const noexcept override { 
		return bool(this->trace_general(this->ray_type(r))); 
	}	
};

template<typename HitType>
class GeneralObject<HitType, Ray> : public Object {
public:
        Ray	                       ray_type(const Ray& r) const noexcept { return r; } //So it still can be called externally.
	virtual Hit                    hit(const Ray& r, const HitType& h) const noexcept = 0;
	virtual float                  hit_distance(const HitType& h) const noexcept = 0;
	virtual std::optional<HitType> trace_general(const Ray& r) const noexcept = 0;

	std::optional<Hit> trace(const Ray& r) const noexcept override {
		std::optional<HitType> h = trace_general(r);
		if (h) return this->hit(r, *h); else return {};
	}
	virtual bool trace_shadow(const Ray& r) const noexcept override { 
		return bool(this->trace_general(r)); 
	}	
};

template<typename RayType>
class GeneralObject<float, RayType> : public Object {
public:
        virtual RayType                ray_type(const Ray& r)     const noexcept = 0; 
	virtual Hit                    hit(const RayType& r, float h) const noexcept = 0;
	virtual float                  hit_distance(float h) const noexcept { return h; }
	virtual std::optional<float>   trace_general(const RayType& r) const noexcept = 0;

	std::optional<Hit> trace(const Ray& r) const noexcept override {
		RayType rt = this->ray_type(r);
		std::optional<float> h = trace_general(rt);
		if (h) return this->hit(rt, *h); else return {};
	}
	virtual bool trace_shadow(const Ray& r) const noexcept override { 
		return bool(this->trace_general(r)); 
	}	
};


template<>
class GeneralObject<float, Ray> : public Object {
public:
        Ray                            ray_type(const Ray& r) const noexcept { return r; } //So it still can be called externally.
	virtual Hit                    hit(const Ray& r, float h) const noexcept = 0;
	virtual float                  hit_distance(float h) const noexcept { return h; }
	virtual std::optional<float>   trace_general(const Ray& r) const noexcept = 0;

	std::optional<Hit> trace(const Ray& r) const noexcept override {
		std::optional<float> h = trace_general(r);
		if (h) return this->hit(r, *h); else return {};
	}
	virtual bool trace_shadow(const Ray& r) const noexcept override { 
		return bool(this->trace_general(r)); 
	}	
};

};
