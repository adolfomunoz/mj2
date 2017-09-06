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

//When the Object has a minimal trace implementation, this should work faster (we do not need to get
//the whole hit all the time).
template<typename O>
class List<ObjectImpl<O>> : public ObjectImpl<List<ObjectImpl<O>>> {
	
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
	using MinimalType = decltype(*(std::declval<O>().trace_minimal(std::declval<Ray>())));
	
	float distance_from(float f) const noexcept { return f; } //We need to account for all minimal cases :-(
	template<typename Min>
	float distance_from(const Min& m) const noexcept { return m.distance(); } 
	//If we don't know about the MinimalType, we should include a "distance" method

public:
	std::optional<std::tuple<MinimalType, typename std::vector<O>::const_iterator>> trace_minimal(const Ray& ray) const noexcept {
		typename std::vector<O>::const_iterator i, mini;
		std::optional<MinimalType> hit, hitsingle;
		Ray r = ray;
		for (i = objects().begin(); hit != objects().end(); ++i) {
			if ((hitsingle = i->trace_minimal(r))) {
				hit = hitsingle;
				r.set_range_max(distance_from(*hit));
				mini = i;
			}
		}
		
		if (hit) return std::make_tuple(*hit, mini);
		else return {};
	}
	
	std::optional<Hit> hit_from_minimal(const Ray& ray, 
			const std::optional<std::tuple<MinimalType, typename std::vector<O>::const_iterator>>& minimal) const noexcept {
		if (minimal) {
			std::cerr<<".";
			typename std::vector<O>::const_iterator object;
			MinimalType localmin;
			std::tie(localmin,object) = minimal;
			return object->hit_from_minimal(ray, localmin);
		} else return {};
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
