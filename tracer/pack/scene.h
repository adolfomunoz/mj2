#pragma once

#include <list>
#include <memory>
#include "../object.h"
#include "list.h"

namespace tracer {
	
class Scene : public List<Object> {
public:
	using List<Object>::List;	
};

};
