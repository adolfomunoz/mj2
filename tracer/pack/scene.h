#pragma once

#include <list>
#include <memory>
#include "../object.h"

namespace tracer {
	
class Scene : public std::list<std::shared_ptr<Object>>, public Object {
public:
	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		std::optional<Hit> hit, hitsingle;
		Ray r = ray;
		for (const std::shared_ptr<Object>& object : (*this)) {
			if ((hitsingle = object->trace(r))) {
				hit = hitsingle;
				r.set_range_max(hit->distance());
			}
		}
		return hit;
	}	
	
};

};
