#pragma once

#include <optional>
#include "ray.h"
#include "hit.h"
#include <memory>

namespace tracer {

class ObjectBase {
#ifdef MATERIAL
protected:
    std::shared_ptr<MATERIAL> mat;
public:
    const std::shared_ptr<MATERIAL>& material() const { return mat; }
#endif

public:
	virtual std::optional<Hit> trace(const Ray& r) const noexcept = 0;
	virtual bool trace_shadow(const Ray& r) const { return bool(trace(r)); }
};

template<typename O>
struct object_traits {
    template<typename U>
    static constexpr auto test_extend_ray(U*)
      -> decltype(U::extend_ray(std::declval<Ray>()));
    template<typename>
    static constexpr auto test_extend_ray(...) -> Ray;

    using RayType = std::decay_t<decltype(test_extend_ray<O>(nullptr))>;
    static constexpr bool has_ray_type = !std::is_same_v<RayType,Ray>;
    using HitType = std::decay_t<decltype(std::declval<O>().trace_general(std::declval<RayType>()).value())>;
    static constexpr bool has_hit_type = !std::is_same_v<HitType,Hit>;
};

constexpr float hit_distance(const Hit& h) noexcept { return h.distance(); }
constexpr float hit_distance(float h) noexcept { return h; }

template<typename O>
class ObjectImpl : public ObjectBase {
public:
#ifdef MATERIAL
    O& set_material(const std::shared_ptr<MATERIAL>& m) { mat = m; return static_cast<O&>(*this); }
    O& set_material(const MATERIAL& m) { return set_material(std::make_shared<MATERIAL>(m)); }
    O& set_material(MATERIAL&& m) { return set_material(std::make_shared<MATERIAL>(std::forward<MATERIAL>(m))); }
#endif

    std::optional<Hit> trace(const Ray& r) const noexcept override {
        if constexpr (object_traits<O>::has_ray_type) {
            auto er = O::extend_ray(r);
            auto h = static_cast<const O*>(this)->trace_general(er);
            if (h) {
                if constexpr (object_traits<O>::has_hit_type) 
                    #ifdef MATERIAL
                        return static_cast<const O*>(this)->hit(er,*h).set_material(this->material());
                    #else
                        return static_cast<const O*>(this)->hit(er,*h);
                    #endif
                else
                    #ifdef MATERIAL
                        return h.set_material(this->material());  
                    #else
                        return h;
                    #endif
            } else return std::optional<Hit>();
        } else {
            auto h = static_cast<const O*>(this)->trace_general(r);
            if (h) {
                if constexpr (object_traits<O>::has_hit_type) 
                    #ifdef MATERIAL
                        return static_cast<const O*>(this)->hit(r,*h).set_material(this->material());
                    #else
                        return static_cast<const O*>(this)->hit(r,*h);
                    #endif
                else
                    #ifdef MATERIAL
                        return h->set_material(this->material());  
                    #else
                        return h;
                    #endif
            } else return std::optional<Hit>();
        }
   }
   
   virtual bool trace_shadow(const Ray& r) const noexcept override {
       if constexpr (object_traits<O>::has_ray_type) {
           auto er = O::extend_ray(r);
           return bool(static_cast<const O*>(this)->trace_general(er));
        } else 
            return bool(static_cast<const O*>(this)->trace_general(r));
   }
};

/**
 * Represents a polymorphic object (holds a smart pointer inside)
 **/
class Object : public ObjectImpl<Object> {
	std::shared_ptr<ObjectBase> o;
public:
	//We assume that O is base of ObjectBase and move it
	template<typename O>
	Object(O&& object) : o(std::make_shared<std::decay_t<O>>(std::forward<O>(object))) {}
	
	Object(const Object& object) : o(object.o) {}
	Object(Object&& object) : o(std::move(object.o)) {}
	
	Object(std::shared_ptr<ObjectBase>&& object) :
		o(std::forward<std::shared_ptr<ObjectBase>>(object)) {}
	Object(const std::shared_ptr<ObjectBase>& object) :
		o(object){}
	
	std::optional<Hit> trace_general(const Ray& r) const noexcept {
		assert(bool(o)); //o should always point to an object
		auto h = o->trace(r);
		#ifdef MATERIAL
		if (h) h->set_material(this->material());
		#endif
		return h;
	}
	bool trace_shadow(const Ray& r) const noexcept override { 
		bool h = false;
		if (o) h = o->trace_shadow(r);
		return h;
	}
};

};
