#pragma once

#include <list>
#include <vector>
#include <initializer_list>
#include "../object.h"

/**
 * This version of list worked slightly slower as ObjectGeneral, and it is highly unlikely that we are going to want to use a list
 * as an ObjectGeneral. Therefore we leave the list as it previously was.
 **/

namespace tracer {

template<typename O>
struct object_traits {
	using RayType = decltype(std::declval<O>().ray_type(std::declval<Ray>()));
	using HitType = typename std::remove_const<typename std::remove_reference<decltype(std::declval<O>().trace_general(std::declval<RayType>()).value())>::type>::type;
};	

//TODO: Deduce HitType and RayType and do this as an "ObjectGeneral<RayType, std::tuple<HitType, int>>" for efficiency purposes.
template<typename O>
class List : public GeneralObject<std::tuple<typename object_traits<O>::HitType,int>, typename object_traits<O>::RayType> {
	static_assert(std::is_base_of_v<Object,O>, "The List is not a list of Objects");
	
	using RayType = typename object_traits<O>::RayType;
	using HitType = typename object_traits<O>::HitType;

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

	//ray_type only overrides if RayType is different than Ray. Otherwise it is not actually overriding anything.
	RayType ray_type(const Ray& ray) const noexcept /*override*/ { return objects()[0].ray_type(ray); }

	float hit_distance(const std::tuple<HitType, int>& h) const noexcept override { return objects()[std::get<1>(h)].hit_distance(std::get<0>(h)); }
	
	Hit hit(const RayType& ray, const std::tuple<HitType, int>& h) const noexcept override {
		return objects()[std::get<1>(h)].hit(ray, std::get<0>(h)); 
	}

	std::optional<std::tuple<HitType, int>> trace_general(const RayType& ray) const noexcept override {
		RayType r = objects()[0].ray_type(ray); //Should be the same for all objects.
		std::optional<HitType> hit, hitsingle;
		int closest_object = -1;
		for (int i = 0; i<objects().size(); ++i) {
			assert(objects()[0].ray_type(ray) == objects()[i].ray_type(ray)); //This assertion is not checked in release mode, but it is neccesary in debug mode.
			if ((hitsingle = objects()[i].trace_general(r))) {
				hit = hitsingle;
				r.set_range_max(objects()[i].hit_distance(*hit));
				closest_object = i;
			}
		}
		if (closest_object>=0) return std::make_tuple(*hit,closest_object);
		else return { };
	}
	
	//Maybe it can be made more efficient (using RayType)?	
	bool trace_shadow(const Ray& ray) const noexcept override {
		for (const O& object : objects()) {
			if (object.trace_shadow(ray))
				return true;
		}
		return false;
	}	
};

}; //namespace tracer
