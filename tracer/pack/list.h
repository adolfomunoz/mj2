#pragma once

#include <list>
#include <vector>
#include <initializer_list>
#include "../object.h"

namespace tracer {

template<typename O>	
struct minimal_hit {
	using type = void;
};

template<typename MinHit>
struct minimal_hit<ObjectMinimal<MinHit>> {
	using type = void;
};

template<typename O>
struct is_minimal : public std::is_void<minimal_hit<O>> { };
	
template<typename O>
class List : public Object {
	static_assert(std::is_base_of_v<Object,O>, "The List is not a list of Objects");
	
	/**
	 * We use a std::vector instead of a std::list for dynamic allocation but locality in memory (more cache-friendlyness)
	 **/
	std::vector<O> objects_; 
	
public:
	//Efficient constructors are not a priority (geometry generation). 
	//Efficient tracing is.
	List(const std::list<O>& il)      noexcept : objects_(il.begin(), il.end()) { }
	List(const std::vector<O>& il)    noexcept : objects_(il.begin(), il.end()) { }
	List(std::vector<O>&& objects)    noexcept : objects_(std::forward<std::vector<O>>(objects)) { } //Efficient moving 
	
	const std::vector<O>& objects() const noexcept { return objects_; }

	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		if constexpr (is_minimal<O>::value) {
			using MinHit = decltype(std::declval<O>().trace_minimal(ray));
			std::optional<MinHit> hit, hitsingle;
			const O& closest_object = *(objects().begin());
			Ray r = ray;
			for (const O& object : objects()) {
				if ((hitsingle = object.trace_minimal(r))) {
					hit = hitsingle;
					r.set_range_max(object.minimal_distance(*hit));
					closest_object = object;
				}
			}
			if (hit) return closest_object.hit_from_minimal(ray, *hit);
			else return {};
		} else {
			std::optional<Hit> hit, hitsingle;
			Ray r = ray;
			for (const O& object : objects()) {
				if ((hitsingle = object.trace(r))) {
					hit = hitsingle;
					r.set_range_max(hit->distance());
				}
			}
			return hit;
		}
	}
	
	bool trace_shadow(const Ray& ray) const noexcept override {
		for (const O& object : objects()) {
			if (object.trace_shadow(ray))
				return true;
		}
		return false;
	}	
};

}; //namespace tracer
