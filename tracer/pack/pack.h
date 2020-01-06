#pragma once

#include "list.h"

namespace tracer {

template<typename O, int N>
class Pack : public List<O> {
public:
	using List<O>::List;

};

template<int N, typename C>
Pack<typename C::value_type,N> pack(const C& l) {
	return Pack<typename C::value_type,N>(l);
}

template<typename O>
Pack<O,2> pack(const O& o1, const O& o2) {
	return Pack<O,2>(std::vector<O>{o1,o2});
}

template<typename O>
Pack<O,3> pack(const O& o1, const O& o2, const O& o3) {
	return Pack<O,3>(std::vector<O>{o1,o2,o3});
}

template<typename O>
Pack<O,4> pack(const O& o1, const O& o2, const O& o3, const O& o4) {
	return Pack<O,4>(std::vector<O>{o1,o2,o3,o4});
}



}
