#pragma once

#include <optional>
#include "ray.h"
#include "hit.h"

namespace tracer {
class Object {
public:
	virtual std::optional<Hit> trace(const Ray& r) const noexcept = 0;
};

};