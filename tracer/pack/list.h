#pragma once

#include <list>
#include <vector>
#include <initializer_list>
#include "../object.h"

namespace tracer {
	
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
	
	bool trace_shadow(const Ray& ray) const noexcept override {
		for (const O& object : objects()) {
			if (object.trace_shadow(ray))
				return true;
		}
		return false;
	}	
};

}; //namespace tracer
