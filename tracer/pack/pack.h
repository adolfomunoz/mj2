#pragma once

#include "list.h"

namespace tracer {

template<typename O, int N>
class Pack : public List<O> {
public:
	using List<O>::List;

};


}
