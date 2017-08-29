#pragma once

namespace tracer {
class Hit {
	float distance_;
//	const Plane& object_;
public:
	Hit(float distance) noexcept :
		distance_(distance) { }
		
	Hit() noexcept : distance_(-1.0f) { }
		
	constexpr float distance() const noexcept { return distance_; }
//	constexpr const Plane& object() const noexcept { return object_; }
	
	constexpr operator bool() const noexcept { return distance()>0.0f; }
};

};