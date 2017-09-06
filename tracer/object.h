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

//CRTP: minimal trace implementation for efficiency
//Requires:
//   std::optional<minimal> trace_minimal(const Ray& ray) const noexcept
//   std::optional<Hit> hit_from_minimal(const Ray& ray, const std::optional<minimal>& minimal)
//minimal can be any data type required for the specific object.
template<typename CRTP>
class ObjectImpl : public Object {
public:
	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		return static_cast<const CRTP*>(this)->hit_from_minimal(
			ray, static_cast<const CRTP*>(this)->trace_minimal(ray));
	}
	
	virtual bool trace_shadow(const Ray& ray) const noexcept override {
		return bool(static_cast<const CRTP*>(this)->trace_minimal(ray));
	}
};

};