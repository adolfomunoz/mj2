#pragma once

#include <list>
#include <vector>
#include <initializer_list>
#include "../object.h"

namespace tracer {

//TODO: Deduce HitType and RayType and do this as an "ObjectGeneral<RayType, std::tuple<HitType, int>>" for efficiency purposes.
template<typename O>
class List : public Object {
	static_assert(std::is_base_of_v<Object,O>, "The List is not a list of Objects");
	
	using RayType = decltype(std::declval<O>().ray_type(std::declval<Ray>()));
	using HitType = typename std::remove_const<typename std::remove_reference<decltype(std::declval<O>().trace_general(std::declval<RayType>()).value())>::type>::type;

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

	//TODO: Add hit_distance
	//TODO: Add hit(RayType,HitType)

	//TODO: Change this to trace_general when Object becomes ObjectGeneral
	std::optional<Hit> trace(const Ray& ray) const noexcept override {
		RayType r = objects()[0].ray_type(ray); //Should be the same for all objects.
		std::optional<HitType> hit, hitsingle;
		const O* closest_object = nullptr;
		for (const O& object : objects()) {
			assert(objects()[0].ray_type(ray) == object.ray_type(ray)); //This assertion is not checked in release mode, but it is neccesary in debug mode.
			if ((hitsingle = object.trace_general(r))) {
				hit = hitsingle;
				r.set_range_max(object.hit_distance(*hit));
				closest_object = &object;
			}
		}
		if (hit) return closest_object->hit(ray,*hit);
		else return { };
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

}; //namespace tracer
