#pragma once

#include <list>
#include "object.h"

namespace tracer {
	
template<typename O>
class List : public Object, public std::list<O> {
	static_assert(std::is_base_of_v<Object,O>, "The List is not a list of Objects");
public:
	using std::list<O>::list;
	
	std::optional<Hit> trace(const Ray& ray) const noexcept {
		std::optional<Hit> hit, hitsingle;
		Ray r = ray;
		for (const Object& object : (*this)) {
			if ((hitsingle = object.trace(r))) {
				hit = hitsingle;
				r.set_range_max(hit->distance());
			}
		}
		return hit;
	}	
	
};

};
