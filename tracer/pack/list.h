#pragma once

#include <list>
#include <vector>
#include <initializer_list>
#include "../object.h"

namespace tracer {
	
template<typename HitType, typename O>
float hit_distance(const std::tuple<HitType,const O*>& h) {
	return hit_distance(std::get<0>(h));
}

//TODO: Deduce HitType and RayType and do this as an "ObjectGeneral<RayType, std::tuple<HitType, int>>" for efficiency purposes.
template<typename O>
class List : public ObjectImpl<List<O>> {
	static_assert(std::is_base_of_v<ObjectBase,O>, "The List is not a list of Objects");
	/**
	 * We use a std::vector instead of a std::list for dynamic allocation but locality in memory (more cache-friendlyness)
	 **/
	std::vector<O> objects_; 
	
	using HitType = typename object_traits<O>::HitType;
	using RayType = typename object_traits<O>::RayType;
public:
	//Efficient constructors are not a priority (geometry generation). 
	//Efficient tracing is.
	List(const std::list<O>& il)      noexcept : objects_(il.begin(), il.end()) { }
	List(const std::vector<O>& il)    noexcept : objects_(il.begin(), il.end()) { }
	List(std::vector<O>&& objects)    noexcept : objects_(std::forward<std::vector<O>>(objects)) { } //Efficient moving 
	List() noexcept {}
	const std::vector<O>& objects() const noexcept { return objects_; }

	void add(const O& o) { objects_.push_back(o); }
	void add(O&& o)      { objects_.push_back(std::forward<O>(o)); }
	//TODO: Add hit_distance
	//TODO: Add hit(RayType,HitType)
	
	static RayType extend_ray(const Ray& r) {
		if constexpr (object_traits<O>::has_ray_type)
			return O::extend_ray(r);
		else
			return r;
	}

	std::optional<std::tuple<HitType,const O*>> 
		trace_general(const typename object_traits<O>::RayType& ray) const noexcept {
			typename object_traits<O>::RayType r = ray;
			std::optional<typename object_traits<O>::HitType> hit, hitsingle;
			const O* closest_object = nullptr;
			for (const O& object : objects()) {
				if ((hitsingle = object.trace_general(r))) {
					hit = hitsingle;
					r.set_range_max(hit_distance(*hit));
					closest_object = &object;
				}
			}
			if (hit) return std::tuple<HitType,const O*>(*hit, closest_object); 
			else return std::optional<std::tuple<HitType,const O*>>();
	}
	
	Hit hit(const RayType& ray, const std::tuple<HitType,const O*>& h) const {
		if constexpr (object_traits<O>::has_hit_type)
			return std::get<1>(h)->hit(ray,std::get<0>(h));
		else
			return std::get<0>(h);	
	}
	
	//TODO: Make more efficient (using RayType)	
	bool trace_shadow(const Ray& ray) const noexcept override {
		for (const O& object : objects()) {
			if (object.trace_shadow(ray))
				return true;
		}
		return false;
	}	
};

template<typename C>
List<typename C::value_type> list(const C& l) {
	return List<typename C::value_type>(l);
}

template<typename O>
List<O> list(const O& o1, const O& o2) {
	return list(std::vector<O>{o1,o2});
}

template<typename O>
List<O> list(const O& o1, const O& o2, const O& o3) {
	return list(std::vector<O>{o1,o2,o3});
}

template<typename O>
List<O> list(const O& o1, const O& o2, const O& o3, const O& o4) {
	return list(std::vector<O>{o1,o2,o3,o4});
}

}; //namespace tracer
