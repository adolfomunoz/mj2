#pragma once

#include <optional>
#include "ray.h"
#include "hit.h"

namespace tracer {
	
class Object {
public:
	virtual std::optional<Hit> trace(const Ray& r) const noexcept = 0;
	virtual bool trace_shadow(const Ray& r) const noexcept { return bool(trace(r)); }
};

/**
 * This is for optimization: MinHit is the minimal representation to obtain a Hit. If
 * there is no hit, just MinHit is computed and not the full hit.
 * This is specially useful for lists or packs of objects, so the Hit is computed
 * only once
 **/ 
template<typename MinHit>
class ObjectMinimal : public Object {
public:
	virtual std::optional<MinHit> trace_minimal(const Ray& ray) const noexcept = 0;
	virtual float minimal_distance(const MinHit& minhit) const noexcept = 0;
	virtual Hit hit_from_minimal(const Ray& ray, const MinHit& minhit) const noexcept = 0;
	
	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		std::optional<MinHit> minhit = trace_minimal(ray);
		if (minhit) return hit_from_minimal(ray, *minhit);
		else return {};
	}
	
	virtual bool trace_shadow(const Ray& ray) const noexcept override {
		return bool(trace_minimal(ray));
	}
};

};